// OGN-Tracker for the CubeCell HELTEC modul with GPS and small OLED.
// Note: requires modifications to the core RF libraries which support OGN transmission and reception

#include "Arduino.h"
#include <Wire.h>

#include "LoRaWan_APP.h"
#include "sx126x.h"

#include "GPS_Air530.h"
#include "GPS_Air530Z.h"

#include "HT_SSD1306Wire.h"
#include "CubeCell_NeoPixel.h"

#include "fifo.h"
#include "lowpass2.h"

#include "format.h"
#include "nmea.h"
#include "manchester.h"
#include "ogn1.h"

#include "freqplan.h"
#include "rfm.h"

static uint64_t getUniqueID(void) { return getID(); }        // get unique serial ID of the CPU/chip
static uint32_t getUniqueAddress(void) { return getID()&0x00FFFFFF; }

#define WITH_OGN1                          // OGN protocol version 1
#define OGN_Packet OGN1_Packet

#define HARDWARE_ID 0x03
#define SOFTWARE_ID 0x01

#define HARD_NAME "CC-OGN"
#define SOFT_NAME "2022.11.15"

#define DEFAULT_AcftType        1         // [0..15] default aircraft-type: glider
#define DEFAULT_GeoidSepar     40         // [m]
#define DEFAULT_CONbaud    115200         // [bps]
#define DEFAULT_PPSdelay      100         // [ms]
#define DEFAULT_FreqPlan        1

#include "parameters.h"

static FlashParameters Parameters;       // parameters stored in Flash: address, aircraft type, etc.

static uint16_t BattVoltage = 0;         // [mV] battery voltage, measured every second

static uint8_t BattCapacity(uint16_t mVolt)
{ if(mVolt>=4050) return 100;                                 // 4.05V or above => full capacity
  if(mVolt<=3550) return   0;                                 // 3.55V or below => zero capacity
  return (mVolt-3550+2)/5; }                                  // linear dependence (simplified)

static uint8_t BattCapacity(void) { return BattCapacity(BattVoltage); }

// ===============================================================================================

static Air530ZClass GPS;                 // GPS

static SSD1306Wire Display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10 ); // OLED: addr , freq , i2c group , resolution , rst

static CubeCell_NeoPixel Pixels(1, RGB, NEO_GRB + NEO_KHZ800);                // three-color LED

// ===============================================================================================

static char Line[256];                                                        // for OLED abd console printing

static void VextON(void)                   // Vext controls OLED power
{ pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW); }

static void VextOFF(void)                  // Vext default OFF
{ pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH); }

static FreqPlan Radio_FreqPlan;       // RF hopping pattern

static FIFO<RFM_FSK_RxPktData, 16> RxFIFO;         // buffer for received packets
static OGN_PrioQueue<OGN1_Packet, 32> RelayQueue;  // candidate packets to be relayed

static Delay<uint8_t, 64> RX_OGN_CountDelay;   // to average the OGN packet rate over one minute
static uint16_t           RX_OGN_Count64=0;    // counts received packets for the last 64 seconds

static LowPass2<int32_t, 4,2,4> RX_RSSI;       // low pass filter to average the RX noise

// ===============================================================================================

static union
{ uint64_t Word;
  struct
  { uint32_t RX;
    uint32_t GPS;
  } ;
} Random = { 0x0123456789ABCDEF };

// ===============================================================================================
// CONSole UART

void CONS_UART_Write(char Byte) // write byte to the console (USB serial port)
{ Serial.write(Byte); }

int  CONS_UART_Free(void)
{ return Serial.availableForWrite(); }

int  CONS_UART_Read (uint8_t &Byte)
{ int Ret=Serial.read(); if(Ret<0) return 0;
  Byte=Ret; return 1; }

// ===============================================================================================

static NMEA_RxMsg ConsNMEA;

static void PrintParameters(void)                               // print parameters stored in Flash
{ Parameters.Print(Line);                                       // single line, most essential parameters
  Format_String(CONS_UART_Write, Line);
  Parameters.Write(CONS_UART_Write); }                          // write the parameters to the console

static void PrintPOGNS(void)                                   // print parameters in the $POGNS form
{ Parameters.WritePOGNS(Line);
  Format_String(CONS_UART_Write, Line);
  Parameters.WritePOGNS_Pilot(Line);
  Format_String(CONS_UART_Write, Line);
  Parameters.WritePOGNS_Acft(Line);
  Format_String(CONS_UART_Write, Line);
  Parameters.WritePOGNS_Comp(Line);
  Format_String(CONS_UART_Write, Line); }

static void ConsNMEA_Process(void)                              // priocess NMEA received on the console
{ if(!ConsNMEA.isPOGNS()) return;                               // ignore all but $POGNS
  if(ConsNMEA.hasCheck() && !ConsNMEA.isChecked() ) return;     // if CRC present then it must be correct
  if(ConsNMEA.Parms==0) { PrintPOGNS(); return; }               // if no parameters given, print the current parameters as $POGNS
  // printf("ConsNMEA_Process() - before .ReadPOGNS()\n\r" );
  Parameters.ReadPOGNS(ConsNMEA);                               // read parameter values given in $POGNS
  // printf("ConsNMEA_Process() - after .ReadPOGNS()\n\r" );
  PrintParameters();                                            // print the new parameter values
  Parameters.WriteToFlash(); }                                  // write new parameter set to flash

static void CONS_CtrlC(void)
{ PrintParameters(); }

static int CONS_Proc(void)
{ int Count=0;
  for( ; ; )
  { uint8_t Byte; int Err=CONS_UART_Read(Byte); if(Err<=0) break;
    Count++;
    if(Byte==0x03) CONS_CtrlC();                                // if Ctrl-C received: print parameters
    ConsNMEA.ProcessByte(Byte);
    // printf("CONS_Proc() Err=%d, Byte=%02X, State/Len=%d/%d\n\r", Err, Byte, ConsNMEA.State, ConsNMEA.Len);
    if(ConsNMEA.isComplete())
    { ConsNMEA_Process();                                           // interpret the ConsNMEA
      // printf("- after _Process()\n\r" );
      ConsNMEA.Clear(); }
    // printf("CONS_Proc() - after if()\n\r" );
  }
  return Count; }

// ===============================================================================================
// Process GPS satelite data

// Satellite count and SNR per system, 0=GPS, 1=GLONASS, 2=GALILEO, 3=BEIDO
static uint16_t SatSNRsum  [4] = { 0, 0, 0, 0 }; // sum up the satellite SNR's
static uint8_t  SatSNRcount[4] = { 0, 0, 0, 0 }; // sum counter

static uint16_t   GPS_SatSNR = 0;                // [0.25dB] average SNR from the GSV sentences
static uint8_t    GPS_SatCnt = 0;                // number of satelites in the above average

static void ProcessGSV(NMEA_RxMsg &GSV)          // process GxGSV to extract satellite data
{ uint8_t SatSys=0;
       if(GSV.isGPGSV()) { SatSys=0; }
  else if(GSV.isGLGSV()) { SatSys=1; }
  else if(GSV.isGAGSV()) { SatSys=2; }
  else if(GSV.isBDGSV()) { SatSys=3; }
  else return;
  if(GSV.Parms<3) return;
  int8_t Pkts=Read_Dec1((const char *)GSV.ParmPtr(0)); if(Pkts<0) return;            // how many packets to pass all sats
  int8_t Pkt =Read_Dec1((const char *)GSV.ParmPtr(1)); if(Pkt <0) return;            // which packet in the sequence
  int8_t Sats=Read_Dec2((const char *)GSV.ParmPtr(2));                               // total number of satellites
  if(Sats<0) Sats=Read_Dec1((const char *)GSV.ParmPtr(2));                           // could be a single or double digit number
  if(Sats<0) return;
  if(Pkt==1) { SatSNRsum[SatSys]=0; SatSNRcount[SatSys]=0; }                         // if 1st packet then clear the sum and counter
  for( int Parm=3; Parm<GSV.Parms; )                                                 // up to 4 sats per packet
  { int8_t PRN =Read_Dec2((const char *)GSV.ParmPtr(Parm++)); if(PRN <0) break;      // PRN number
    int8_t Elev=Read_Dec2((const char *)GSV.ParmPtr(Parm++)); if(Elev<0) break;      // [deg] eleveation
   int16_t Azim=Read_Dec3((const char *)GSV.ParmPtr(Parm++)); if(Azim<0) break;      // [deg] azimuth
    int8_t SNR =Read_Dec2((const char *)GSV.ParmPtr(Parm++)); if(SNR<=0) continue;   // [dB] SNR or absent when not tracked
    SatSNRsum[SatSys]+=SNR; SatSNRcount[SatSys]++; }                                 // add up SNR
  if(Pkt==Pkts)                                                                      // if the last packet
  { uint8_t Count=0; uint16_t Sum=0;
    for(uint8_t Sys=0; Sys<4; Sys++)
    { if(SatSNRcount[Sys]==0) continue;
      Count+=SatSNRcount[Sys]; Sum+=SatSNRsum[Sys]; }
    GPS_SatCnt = Count;
    if(Count) GPS_SatSNR = (4*Sum+Count/2)/Count;
        else  GPS_SatSNR = 0;
  }
}

// ===============================================================================================
// Process GPS position data

static int          GPS_Ptr = 0;       // 
static GPS_Position GPS_Pipe[2];       // two most recent GPS readouts

static uint32_t GPS_Latitude;          // [1/60000deg]
static uint32_t GPS_Longitude;         // [1/60000deg]
static uint32_t GPS_Altitude;          // [0.1m]
static  int16_t GPS_GeoidSepar= 0;     // [0.1m]
static uint16_t GPS_LatCosine = 3000;  // [1.0/4096]
static uint8_t  GPS_Satellites = 0;    //

static uint32_t GPS_PPS_ms = 0;                        // [ms] System timer at the most recent PPS
static uint32_t GPS_PPS_Time = 0;                      // [sec] Unix time which corresponds to the most recent PPS

static uint32_t GPS_Idle=0;                            // [ticks] to detect when GPS stops sending data

static NMEA_RxMsg GpsNMEA;                             // NMEA catcher

static int GPS_Process(void)                           // process serial data stream from the GPS
{ int Count=0;
  for( ; ; )
  { if(GPS.available()<=0) break;                      // if no more characters then give up
    uint8_t Byte=GPS.read();                           // read character
    // GPS.encode(Byte);                                 // process character through the GPS NMEA interpreter
    GpsNMEA.ProcessByte(Byte);                         // NMEA interpreter
    if(GpsNMEA.isComplete())                           // if NMEA is done
    { if(GpsNMEA.isGxGSV()) ProcessGSV(GpsNMEA);       // process satellite data
       else GPS_Pipe[GPS_Ptr].ReadNMEA(GpsNMEA);       // interpret the position NMEA by the GPS
      GpsNMEA.Clear(); }
    Serial.write(Byte);                                // copy character to the console (we could copy only the selected and correct sentences)
    Count++; }                                         // count processed characters
  return Count; }                                      // return number of processed characters

static void GPS_Next(void)                             // step to the next GPS position
{ int Next = GPS_Ptr^1;
  if(GPS_Pipe[GPS_Ptr].isValid() && GPS_Pipe[Next].isValid()) GPS_Pipe[GPS_Ptr].calcDifferentials(GPS_Pipe[Next]);
  GPS_Ptr=Next; }

static void GPS_Random_Update(uint8_t Bit)             // process single LSB bit from the GPS data
{ Random.GPS = (Random.GPS<<1) | (Bit&1); }

static void GPS_Random_Update(const GPS_Position &Pos) // process LSB bits to produce random number
{ GPS_Random_Update(Pos.Altitude);
  GPS_Random_Update(Pos.Speed);
  GPS_Random_Update(Pos.Latitude);
  GPS_Random_Update(Pos.Longitude);
  XorShift32(Random.GPS); }

// ===============================================================================================

static void OLED_Logo(void)                                               // display the logo page
{ Display.clear();

  Display.drawCircleQuads(96, 32, 30, 0b1111);
  Display.drawCircleQuads(96, 32, 34, 0b0001);
  Display.drawCircleQuads(96, 32, 38, 0b0001);

  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  // u8g2_SetFont(OLED, u8g2_font_ncenB14_tr);
  Display.drawString(96, 14, "OGN");
  // u8g2_SetFont(OLED, u8g2_font_8x13_tr);
  Display.drawString(96, 26, "Tracker");

  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString( 0,  0, "CC-OGN");
  // Display.drawString( 0, 38, "");

  Display.setFont(ArialMT_Plain_10);
  Parameters.Print(Line); Line[10]=0;
  Display.drawString( 0, 16, Line);
#ifdef SOFT_NAME
  Display.drawString( 0, 54, SOFT_NAME);
#endif
  Display.display(); }

static void OLED_Info(void)                                               // display the information page
{ Display.clear();
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  uint8_t VertPos=0;
  Parameters.Print(Line); Line[10]=0;                                     // display AcftType:AddrType:Address for identification
  Display.drawString(0, VertPos, Line); VertPos+=16;
  for(uint8_t Idx=0; Idx<Parameters.InfoParmNum; Idx++)                   // loop over parameters and display those non-empty
  { if(Parameters.InfoParmValue(Idx)[0])                                  // if parameter not empty
    { uint8_t Len=Format_String(Line, OGN_Packet::InfoParmName(Idx));     // format parameter name
      Line[Len++]=':'; Line[Len++]=' ';
      Len+=Format_String(Line+Len, Parameters.InfoParmValue(Idx));        // parameter value
      Line[Len]=0;
      Display.drawString(0, VertPos, Line); VertPos+=16; }                // display the line on the OLED
    if(VertPos>=64) break; }                                              // give up when out of display vertical range
  Display.display(); }

static void OLED_GPS(const GPS_Position &GPS)                 // display time, date and GPS data/status
{ Display.clear();
  Display.setFont(ArialMT_Plain_16);
  if(GPS.isTimeValid())
  { uint8_t Len = GPS.FormatDate(Line, '.'); }
  else { strcpy(Line, " no date "); }
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.drawString(128, 0, Line);                           // 1st line right corner: date
  if(GPS.isTimeValid() && GPS.isDateValid())
  { uint8_t Len = GPS.FormatTime(Line, ':');
    Line[Len-4] = 0; }
  else { strcpy(Line, " no time "); }
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 0, Line);                             // 1st line left corner: time
  if(GPS.isValid())
  { uint8_t Len=0;
    Len+=Format_SignDec(Line+Len, GPS.Latitude/60, 6, 4); Line[Len]=0;
    Display.setTextAlignment(TEXT_ALIGN_LEFT);
    Display.drawString(0, 16, Line);
    Len=0;
    Len+=Format_SignDec(Line+Len, GPS.Longitude/60, 7, 4); Line[Len]=0;
    Display.drawString(0, 32, Line);
    Len=0;
    Len+=Format_SignDec(Line+Len, GPS.Altitude/10, 1, 0, 1);
    Line[Len++]='m'; Line[Len]=0;
    Display.setTextAlignment(TEXT_ALIGN_RIGHT);
    Display.drawString(128, 16, Line); }
  { uint8_t Len=0;
    if(GPS.Sec&1)
    { if(GPS.isValid()) Len+=Format_UnsDec(Line+Len, GPS.Satellites);
                  else  Len+=Format_UnsDec(Line+Len, GPS_SatCnt);
      Line[Len++]='s'; Line[Len++]='a'; Line[Len++]='t'; }
    else
    { Len+=Format_UnsDec(Line+Len, (GPS_SatSNR+2)/4, 2);
      Line[Len++]='d'; Line[Len++]='B'; }
    Line[Len]=0;
    Display.setTextAlignment(TEXT_ALIGN_RIGHT);
    Display.drawString(128, 32, Line); }
  { uint8_t Len=0;
/*
    if(GPS.Sec&1)
    { Len+=Format_UnsDec(Line+Len, RX_OGN_Count64);
      Len+=Format_String(Line+Len, "/min"); }
    else
    { Len+=Format_SignDec(Line+Len, RX_RSSI.getOutput()*5, 2, 1);
      Len+=Format_String(Line+Len, "dBm"); }
    Line[Len]=0;
    Display.setTextAlignment(TEXT_ALIGN_LEFT);
    Display.drawString(0, 48, Line);                           // 4th line: number of aircrafts and battery voltage
    Len=0;
*/
    if(GPS.Sec&1) { Len+=Format_UnsDec(Line+Len, (BattVoltage+5)/10, 3, 2); Line[Len++]= 'V'; }
    else
    { uint8_t Cap = BattCapacity();
      Len+=Format_UnsDec(Line+Len, Cap); Line[Len++]= '%'; }
    Line[Len]=0;
    Display.setTextAlignment(TEXT_ALIGN_RIGHT);
    Display.drawString(128, 48, Line); }
  Display.display(); }

static void OLED_RF(void)                 // display RF-related data
{ Display.clear();
  Display.setFont(ArialMT_Plain_16);

  uint8_t Len=Format_String(Line, "SX1262: ");
  Len+=Format_SignDec(Line+Len, (int16_t)Parameters.TxPower);              // Tx power
  Len+=Format_String(Line+Len, "dBm");
  Line[Len]=0;
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 0, Line);

  Len=0; // Len=Format_String(Line, "Rx: ");
  Len+=Format_SignDec(Line+Len, RX_RSSI.getOutput()*5, 2, 1);
  Len+=Format_String(Line+Len, "dBm");
  Line[Len]=0;
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.drawString(128, 16, Line);

  Len=0;
  Len+=Format_UnsDec(Line+Len, RelayQueue.size());
  Len+=Format_String(Line+Len, " Acft");
  Line[Len]=0;
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 32, Line);

  Len=0;
  Len+=Format_UnsDec(Line+Len, RX_OGN_Count64);
  Len+=Format_String(Line+Len, "/min");
  Line[Len]=0;
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.drawString(128, 32, Line);

  Len=0;
  Len+=Format_String(Line+Len, Radio_FreqPlan.getPlanName());                 // name of the frequency plan
  Line[Len++]=' ';
  Len+=Format_UnsDec(Line+Len, (uint16_t)(Radio_FreqPlan.getCenterFreq()/100000), 3, 1); // center frequency
  Len+=Format_String(Line+Len, "MHz");
  Line[Len]=0;
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 48, Line);

  Display.display(); }

static int OLED_CurrPage = 0;

static void OLED_NextPage(void)
{ OLED_CurrPage++;
  if(OLED_CurrPage>=3) OLED_CurrPage=0; }

static void OLED_DispPage(const GPS_Position &GPS)
{ switch(OLED_CurrPage)
  { case 2: OLED_Info(); break; 
    case 1: OLED_RF(); break;
    default: OLED_GPS(GPS); }
}

// ===============================================================================================
// RGB-LED

static void LED_OFF   (void) { Pixels.setPixelColor( 0,   0,   0,   0, 0); Pixels.show(); }
static void LED_Red   (void) { Pixels.setPixelColor( 0, 255,   0,   0, 0); Pixels.show(); }
static void LED_Orange(void) { Pixels.setPixelColor( 0, 255,  96,   0, 0); Pixels.show(); }
static void LED_Green (void) { Pixels.setPixelColor( 0,   0,  96,   0, 0); Pixels.show(); }
static void LED_Yellow(void) { Pixels.setPixelColor( 0, 255,  96,   0, 0); Pixels.show(); }
static void LED_Blue  (void) { Pixels.setPixelColor( 0,   0,   0, 255, 0); Pixels.show(); }

// ===============================================================================================
// OGN packets

static uint16_t InfoParmIdx = 0;                                       // the round-robin index to info records in info packets

static int ReadInfo(OGN1_Packet &Packet)
{ Packet.clrInfo();
  uint8_t ParmIdx;
  for( ParmIdx=InfoParmIdx; ; )
  { const char *Parm = Parameters.InfoParmValue(ParmIdx);
    if(Parm)
    { if(Parm[0])
      { int Add=Packet.addInfo(Parm, ParmIdx); if(Add==0) break; }
    }
    ParmIdx++; if(ParmIdx>=Parameters.InfoParmNum) ParmIdx=0;
    if(ParmIdx==InfoParmIdx) break;
  }
  InfoParmIdx = ParmIdx;
  Packet.setInfoCheck();
  return Packet.Info.DataChars; }                                      // zero => no info parameters were stored

static int getInfoPacket(OGN1_Packet &Packet)                          // produce info-type OGN packet
{ Packet.HeaderWord = 0;
  Packet.Header.Address = Parameters.Address;                          // aircraft address
  Packet.Header.AddrType = Parameters.AddrType;                        // aircraft address-type
  Packet.Header.NonPos = 1;
  Packet.calcAddrParity();
  return ReadInfo(Packet); }                                           // encode the info from the parameters

static int getStatusPacket(OGN1_Packet &Packet, const GPS_Position &GPS)
{ Packet.HeaderWord = 0;
  Packet.Header.Address = Parameters.Address;                          // aircraft address
  Packet.Header.AddrType = Parameters.AddrType;                        // aircraft address-type
  Packet.Header.NonPos = 1;
  Packet.calcAddrParity();
  Packet.Status.Hardware=HARDWARE_ID;
  Packet.Status.Firmware=SOFTWARE_ID;
  GPS.EncodeStatus(Packet);
  uint8_t SatSNR = (GPS_SatSNR+2)/4;                            // encode number of satellites and SNR in the Status packet
  if(SatSNR>8) { SatSNR-=8; if(SatSNR>31) SatSNR=31; }
          else { SatSNR=0; }
  Packet.Status.SatSNR = SatSNR;
  Packet.clrTemperature();
  Packet.EncodeVoltage(((BattVoltage<<3)+62)/125);              // [1/64V]
  Packet.Status.RadioNoise = -RX_RSSI.getOutput();              // [-0.5dBm]
  uint16_t RxRate = RX_OGN_Count64+1;
  uint8_t RxRateLog2=0; RxRate>>=1; while(RxRate) { RxRate>>=1; RxRateLog2++; }
  Packet.Status.RxRate = RxRateLog2;
  uint8_t TxPower = Parameters.TxPower-4;
  if(TxPower>15) TxPower=15;
  Packet.Status.TxPower = TxPower;
  return 1; }

static int getPosPacket(OGN1_Packet &Packet, const GPS_Position &GPS)  // produce position OGN packet
{ Packet.HeaderWord = 0;
  Packet.Header.Address = Parameters.Address;                          // aircraft address
  Packet.Header.AddrType = Parameters.AddrType;                        // aircraft address-type
  Packet.calcAddrParity();
  Packet.Position.AcftType = Parameters.AcftType;                      // aircraft type
  Packet.Position.Stealth  = Parameters.Stealth;
  GPS.Encode(Packet);
  return 1; }

// ===============================================================================================
// Radio

// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
static const uint8_t OGN1_SYNC[10] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A, 0x00, 0x00 };
// OGNv2 SYNC:       0xF56D3738 encoded in Machester
// static const uint8_t OGN2_SYNC[10] = { 0x55, 0x99, 0x96, 0x59, 0xA5, 0x95, 0xA5, 0x6A, 0x00, 0x00 };

// static const uint8_t PAW_SYNC [10] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x18, 0x71, 0x00, 0x00 };

static bool RF_Slot = 0;       // 0 = first TX/RX slot, 1 = second TX/RX slot
static uint8_t RF_Channel = 0; // hopping channel

static RadioEvents_t Radio_Events;

static void Radio_TxDone(void)
{ // Serial.printf("%d: Radio_TxDone()\n", millis());
  Radio.Rx(0); }

static void Radio_TxTimeout(void)
{ // Serial.printf("%d: Radio_TxTimeout()\n", millis());
  Radio.Rx(0); }

static uint8_t RX_OGN_Packets=0;            // [packets] counts received packets

static LDPC_Decoder      Decoder;      // decoder and error corrector for the OGN Gallager/LDPC code
static RFM_FSK_RxPktData RxPktData;

static void CleanRelayQueue(uint32_t Time, uint32_t Delay=20) // remove "old" packets from the relay queue
{ RelayQueue.cleanTime((Time-Delay)%60); }            // remove packets 20(default) seconds into the past

static bool GetRelayPacket(OGN_TxPacket<OGN_Packet> *Packet)      // prepare a packet to be relayed
{ if(RelayQueue.Sum==0) return 0;                     // if no packets in the relay queue
  XorShift32(Random.RX);                              // produce a new random number
  uint8_t Idx=RelayQueue.getRand(Random.RX);          // get weight-random packet from the relay queue
  if(RelayQueue.Packet[Idx].Rank==0) return 0;        // should not happen ...
  memcpy(Packet->Packet.Byte(), RelayQueue[Idx]->Byte(), OGN_Packet::Bytes); // copy the packet
  Packet->Packet.Header.Relay=1;                      // increment the relay count (in fact we only do single relay)
  // Packet->Packet.calcAddrParity();
  if(!Packet->Packet.Header.Encrypted) Packet->Packet.Whiten(); // whiten but only for non-encrypted packets
  Packet->calcFEC();                                  // Calc. the FEC code => packet ready for transmission
  // PrintRelayQueue(Idx);  // for debug
  RelayQueue.decrRank(Idx);                           // reduce the rank of the packet selected for relay
  return 1; }

// a new packet has been received callback - this should probably be a quick call
static void Radio_RxDone( uint8_t *Packet, uint16_t Size, int16_t RSSI, int8_t SNR) // RSSI and SNR are not passed for FSK packets
{ if(Size!=2*26) return;
  RX_OGN_Packets++;
  LED_Green();                                                         // green flash
  PacketStatus_t RadioPktStatus; // to get the packet RSSI: https://github.com/HelTecAutomation/CubeCell-Arduino/issues/236
  SX126xGetPacketStatus(&RadioPktStatus);
  RSSI = RadioPktStatus.Params.Gfsk.RssiAvg;
  RFM_FSK_RxPktData *RxPkt = RxFIFO.getWrite();                        // new packet in the RxFIFO
  RxPkt->Time = GPS_PPS_Time;                                          // [sec]
  RxPkt->msTime = millis()-GPS_PPS_ms;                                 // [ms] time since PPS
  RxPkt->Channel = 0x80 | RF_Channel;                                  // system:channel
  RxPkt->RSSI = -2*RSSI;                                               // [-0.5dBm]
  uint8_t PktIdx=0;
  for(uint8_t Idx=0; Idx<26; Idx++)                                    // loop over packet bytes
  { uint8_t ByteH = Packet[PktIdx++];
    ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode manchester, detect (some) errors
    uint8_t ByteL = Packet[PktIdx++];
    ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F;
    RxPkt->Data[Idx]=(ByteH<<4) | ByteL;
    RxPkt->Err [Idx]=(ErrH <<4) | ErrL ; }
  RxFIFO.Write();                                                      // put packet into the RxFIFO
  Random.RX = (Random.RX*RSSI) ^ (~RSSI); XorShift32(Random.RX);       // update random number
  LED_OFF(); }

static void Radio_RxProcess(void)                                      // process packets in the RxFIFO
{ RFM_FSK_RxPktData *RxPkt = RxFIFO.getRead();                         // check for new received packets
  if(RxPkt==0) return;
  uint8_t RxPacketIdx  = RelayQueue.getNew();                          // get place for this new packet
  OGN_RxPacket<OGN1_Packet> *RxPacket = RelayQueue[RxPacketIdx];
  uint8_t DecErr = RxPkt->Decode(*RxPacket, Decoder);                  // LDPC FEC decoder
  // Serial.printf("%d: Radio_RxDone( , %d, %d, %d) RxErr=%d/%d %08X { %08X %08X }\n",
  //         millis(), Size, RSSI, SNR, DecErr, RxPacket->RxErr, RxPacket->Packet.HeaderWord, Random.GPS, Random.RX);
  if(DecErr || RxPacket->RxErr>=10 ) return;                           // if FEC not correctly decoded or too many bit errors then give up
  uint8_t OwnPacket = ( RxPacket->Packet.Header.Address  == Parameters.Address  )       // is it my own packet (through a relay) ?
                   && ( RxPacket->Packet.Header.AddrType == Parameters.AddrType );
  if(OwnPacket || RxPacket->Packet.Header.NonPos || RxPacket->Packet.Header.Encrypted) { LED_OFF(); return; }
  RxPacket->Packet.Dewhiten();
  if(GPS_Satellites)
  { int32_t LatDist=0, LonDist=0;
    bool DistOK = RxPacket->Packet.calcDistanceVector(LatDist, LonDist, GPS_Latitude, GPS_Longitude, GPS_LatCosine)>=0;
    if(DistOK)
    { RxPacket->calcRelayRank(GPS_Altitude/10);                        // calculate the relay-rank (priority for relay)
      OGN_RxPacket<OGN1_Packet> *PrevRxPacket = RelayQueue.addNew(RxPacketIdx);
      // uint8_t Len=RxPacket->WritePOGNT(Line);
    }
  }
  // Serial.printf("%d: Radio_RxDone( , %d, %d, %d) RxErr=%d %08X\n", millis(), Size, RSSI, SNR, RxPacket->RxErr, RxPacket->Packet.HeaderWord);
  // RxPktData->Print(CONS_UART_Write, 1);
  RxFIFO.Read(); }

extern SX126x_t SX126x; // access to LoraWan102 driver parameters in LoraWan102/src/radio/radio.c

static void OGN_UpdateConfig(const uint8_t *SyncWord, uint8_t SyncBytes) // RF configuration reuired for OGN to work
{ SX126x.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_05;
  SX126x.PacketParams.Params.Gfsk.SyncWordLength = SyncBytes*8;
  SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREE_OFF;
  SX126xSetPacketParams(&SX126x.PacketParams);
  SX126xSetSyncWord((uint8_t *)SyncWord); }

static void OGN_TxConfig(void)
{ Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower, 50000, 0, 100000, 0, 1, 1, 0, 0, 0, 0, 20);
  OGN_UpdateConfig(OGN1_SYNC, 8); }

// static void PAW_TxConfig(void)      // incorrect because the PAW preamble needs to be long and the library does not support it
// { Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower+6, 9600, 0, 38400, 0, 1, 1, 0, 0, 0, 0, 20, 8, PAW_SYNC); }

static void OGN_RxConfig(void)
{ Radio.SetRxConfig(MODEM_FSK, 250000, 100000, 0, 250000, 1, 100, 1, 52, 0, 0, 0, 0, true);
  OGN_UpdateConfig(OGN1_SYNC+1, 7); }

static int OGN_Transmit(const uint8_t *Data, uint8_t Len=26)      // send packet, but first manchester encode it
{ uint8_t Packet[2*Len];
  uint8_t PktIdx=0;
  for(uint8_t Idx=0; Idx<Len; Idx++)
  { uint8_t Byte=Data[Idx];
    Packet[PktIdx++]=ManchesterEncode[Byte>>4];                   // software manchester encode every byte
    Packet[PktIdx++]=ManchesterEncode[Byte&0x0F];
  }
  Radio.Send(Packet, 2*Len);
  return 0; }

static int OGN_Transmit(OGN_TxPacket<OGN1_Packet> &TxPacket)
{ return OGN_Transmit(TxPacket.Byte(), TxPacket.Bytes); }

// ===============================================================================================

static void Sleep(void)
{ GPS.end();
  Radio.Sleep();
  Display.stop();
  LED_OFF(); // turnOffRGB();
  Wire.end();
  Serial.end();
  detachInterrupt(RADIO_DIO_1);
  pinMode(Vext, ANALOG);
  pinMode(ADC, ANALOG);
  while(1) lowPowerHandler();
}

static bool Button_isPressed(void) { return digitalRead(USER_KEY)==0; }

static uint32_t Button_PrevSysTime=0;               // previous sys-time when the Button_Process() was called
static uint32_t Button_PressTime=0;                 // count the time the button is pressed
static bool Button_LowPower=0;

static void Button_Process(void)
{ uint32_t SysTime = millis();                      // [ms]
  uint32_t Diff = SysTime-Button_PrevSysTime;       // [ms] since previous call
  if(!Button_isPressed())                           // if button not pressed (any more)
  { if(Button_PressTime>=100) OLED_NextPage();      // switch to the next OLED page
    Button_PressTime=0;                             // clear the press-time counter
    Button_LowPower=0; }                            // reset counter to enter sleep
  else                                              // when button is pressed
  { Button_PressTime += Diff;                       // accumulate the press-time
    if(Button_PressTime>=1000) Button_LowPower=1;   // if more than one second then declare low-power request
  }
  Button_PrevSysTime=SysTime; }

// ===============================================================================================

void setup()
{ // delay(2000); // prevents USB driver crash on startup, do not omit this

  VextON();                               // Turn on power to OLED (and possibly other external devices)
  delay(100);

  Parameters.ReadFromFlash();             // read parameters from Flash
#ifdef HARD_NAME
  strcpy(Parameters.Hard, HARD_NAME);
#endif
#ifdef SOFT_NAME
  strcpy(Parameters.Soft, SOFT_NAME);
#endif

  Serial.begin(Parameters.CONbaud);       // Start console/debug UART
  Serial.setRxBufferSize(120);            // buffer on RX as there is no multitasking
  // Serial.setTxBufferSize(128);
  // while (!Serial) { }                  // wait for USB serial port to connect

  Serial.println("OGN Tracker on HELTEC CubeCell with GPS");

  pinMode(USER_KEY, INPUT);               // push button

  Pixels.begin();                         // Start RGB LED
  Pixels.clear();
  Pixels.setBrightness(0x20);
  LED_Green();
  // Pixels.setPixelColor( 0, 255, 0, 0, 0); // Green
  // Pixels.show();

  Display.init();                         // Start the OLED
  OLED_Logo();
  // OLED_Info();
  GPS.begin(57600);                                  // Start the GPS

  Radio_FreqPlan.setPlan(Parameters.FreqPlan);       // set the frequency plan from the parameters

  Radio_Events.TxDone    = Radio_TxDone;             // Start the Radio
  Radio_Events.TxTimeout = Radio_TxTimeout;
  Radio_Events.RxDone    = Radio_RxDone;
  Radio.Init(&Radio_Events);
  Radio.SetChannel(Radio_FreqPlan.getFrequency(0));
  OGN_TxConfig();
  OGN_RxConfig();
  Radio.Rx(0);

  RX_RSSI.Set(-2*110);
}

static OGN_TxPacket<OGN1_Packet> TxPosPacket, TxStatPacket, TxRelPacket, TxInfoPacket;

static bool GPS_Done = 0;      // State: 1 = GPS is sending data, 0 = GPS sent all data, waiting for the next PPS

static uint32_t TxTime0, TxTime1; // transmision times for the two slots
static OGN_TxPacket<OGN1_Packet> *TxPkt0, *TxPkt1;

static int32_t  RxRssiSum=0;                // sum RSSI readouts
static int      RxRssiCount=0;              // count RSSI readouts

static void StartRFslot(void)                                     // start the TX/RX time slot right after the GPS stops sending data
{ if(RxRssiCount) { RX_RSSI.Process(RxRssiSum/RxRssiCount); RxRssiSum=0; RxRssiCount=0; }

  XorShift64(Random.Word);
  GPS_Position &GPS = GPS_Pipe[GPS_Ptr];
  GPS_Satellites = GPS.Satellites;
  if(GPS.isTimeValid() && GPS.isDateValid()) { LED_Blue(); GPS_PPS_Time = GPS.getUnixTime(); }    // if time and date are valid
                                       else  { LED_Yellow(); }
  RX_OGN_Count64 += RX_OGN_Packets - RX_OGN_CountDelay.Input(RX_OGN_Packets); // add OGN packets received, subtract packets received 64 seconds ago
  RX_OGN_Packets=0;                                                           // clear the received packet count
  CleanRelayQueue(GPS_PPS_Time);
  if(GPS.isValid())                                           // if position is valid
  { GPS_Altitude  = GPS.Altitude;                             // set global GPS variables
    GPS_Latitude  = GPS.Latitude;
    GPS_Longitude = GPS.Longitude;
    GPS_GeoidSepar= GPS.GeoidSeparation;
    GPS_LatCosine = GPS.LatitudeCosine;
    Radio_FreqPlan.setPlan(GPS_Latitude, GPS_Longitude);      // set Radio frequency plan
    GPS_Random_Update(GPS);
    getPosPacket(TxPosPacket.Packet, GPS);                    // produce position packet to be transmitted
    TxPosPacket.Packet.Whiten();
    TxPosPacket.calcFEC();                                    // position packet is ready for transmission
  }
  CONS_Proc();
  BattVoltage = getBatteryVoltage();                          // [mv] measure the battery voltage (average over 50 readouts)
  // analogReadmV(ADC1);                                      // [4mV] a single readout (see HELTEC library for details)
  CONS_Proc();
  OLED_DispPage(GPS);                                         // display GPS data or other page on the OLED
  CONS_Proc();
  RF_Slot=0;
  RF_Channel=Radio_FreqPlan.getChannel(GPS_PPS_Time, RF_Slot, 1);
  Radio.SetChannel(Radio_FreqPlan.getChanFrequency(RF_Channel));
  OGN_TxConfig();
  OGN_RxConfig();
  Radio.Rx(0);
  TxTime0 = Random.RX  % 399; TxTime0 += 400;
  TxTime1 = Random.GPS % 399; TxTime1 += 800;
  TxPkt0=TxPkt1=0;
  if(GPS.isValid()) TxPkt0 = TxPkt1 = &TxPosPacket;
  static uint8_t InfoTxBackOff=0;
  static uint8_t InfoToggle=0;
  if(InfoTxBackOff) InfoTxBackOff--;
  else
  { InfoToggle = !InfoToggle;
    int Ret=0;
    if(InfoToggle) Ret=getInfoPacket(TxInfoPacket.Packet);
    if(Ret<=0) Ret=getStatusPacket(TxInfoPacket.Packet, GPS);
    if(Ret>0)
    { TxInfoPacket.Packet.Whiten(); TxInfoPacket.calcFEC();
      if(Random.RX&0x10) TxPkt1 = &TxInfoPacket;
                    else TxPkt0 = &TxInfoPacket;
      InfoTxBackOff = 15 + (Random.RX%3);
    }
  }
  static uint8_t RelayTxBackOff=0;
  if(RelayTxBackOff) RelayTxBackOff--;
  else if(GetRelayPacket(&TxRelPacket))
  { if(Random.RX&0x20) TxPkt1 = &TxRelPacket;
                  else TxPkt0 = &TxRelPacket;
    RelayTxBackOff = Random.RX%3; }
  LED_OFF();
  GPS_Next(); }

void loop()
{
  Button_Process();                                               // check for button short/long press
  if(Button_LowPower) { Sleep(); return; }

  Radio_RxProcess();                                              // process received packets, if any

  CONS_Proc();                                                    // process input from the console
  if(GPS_Process()==0) { GPS_Idle++; delay(1); }                  // process input from the GPS
                  else { GPS_Idle=0; RxRssiSum+=2*Radio.Rssi(MODEM_FSK); RxRssiCount++; } // [0.5dBm]
  if(GPS_Done)                                                    // if state is GPS not sending data
  { if(GPS_Idle<3)                                                // GPS (re)started sending data
    { GPS_Done=0;                                                 // change the state to GPS is sending data
      GPS_PPS_ms=millis()-Parameters.PPSdelay;                    // record the est. PPS time
      GPS_PPS_Time++; }
  }
  else
  { if(GPS_Idle>5)                                               // GPS stopped sending data
    { StartRFslot();                                             // start the RF slot
      GPS_Done=1;
    }
  }

  uint32_t SysTime=millis();
  if(RF_Slot==0)                                                 // 1st half of the second
  { if(TxPkt0 && SysTime >= (GPS_PPS_ms+TxTime0))                //
    { // printf("TX   #0: %d\n", SysTime);
      OGN_Transmit(*TxPkt0); TxPkt0=0; }
    else if(SysTime >= (GPS_PPS_ms+800))
    { RF_Slot=1;
      RF_Channel=Radio_FreqPlan.getChannel(GPS_PPS_Time, RF_Slot, 1);
      Radio.SetChannel(Radio_FreqPlan.getChanFrequency(RF_Channel));
      Radio.Rx(0);
      // printf("Slot #1: %d\n", SysTime);
    }
  } else                                                         // 2nd half of the second
  { if(TxPkt1 && SysTime >= (GPS_PPS_ms+TxTime1))
    { // printf("TX   #1: %d\n", SysTime);
      OGN_Transmit(*TxPkt1); TxPkt1=0; }
  }

}
