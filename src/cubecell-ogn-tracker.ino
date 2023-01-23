// OGN-Tracker for the CubeCell HELTEC modul with GPS and small OLED.

// #define WITH_ADSL // needs -O2 compiler flag, otherwise XXTEA gets stuck, not understood why
// #define WITH_BME280 // read a BME280 pressure/temperature/humidity sensor attached to the I2C (same as the OLED)
// #define WITH_BMP280 // read a BMP280 pressure/temperature/humidity sensor attached to the I2C (same as the OLED)

#include "Arduino.h"
#include <Wire.h>

#include "LoRaWan_APP.h"
#include "sx126x.h"

#include "GPS_Air530Z.h"

#include "HT_SSD1306Wire.h"
#include "CubeCell_NeoPixel.h"

#ifdef WITH_BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>  // pressure/temperature/humidity sensor
#endif

#ifdef WITH_BMP280
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>  // pressure/temperature/humidity sensor
#endif

#include "fifo.h"
#include "lowpass2.h"
#include "atmosphere.h"
#include "format.h"
#include "nmea.h"
#include "manchester.h"
#include "ogn1.h"
#include "adsl.h"
#include "crc1021.h"
#include "freqplan.h"
#include "rfm.h"

// ===============================================================================================
// #define WITH_DIG_SIGN

#ifdef WITH_DIG_SIGN
#include "uecc-signkey.h"

static uECC_SignKey SignKey;
#endif

// ===============================================================================================
// DEBUG pin

#define WITH_DEBUGPIN

#ifdef WITH_DEBUGPIN
const int DebugPin = GPIO4; // marked "4" on the board

static void DebugPin_Init(void) { pinMode(DebugPin, OUTPUT); }

static void DebugPin_ON(bool ON=1) { digitalWrite(DebugPin, ON); }
#endif

// ===============================================================================================

static uint64_t getUniqueID(void) { return getID(); }        // get unique serial ID of the CPU/chip
static uint32_t getUniqueAddress(void) { return getID()&0x00FFFFFF; }

#define WITH_OGN1                          // OGN protocol version 1
#define OGN_Packet OGN1_Packet

#define HARDWARE_ID 0x03
#define SOFTWARE_ID 0x01

#define HARD_NAME "CC-OGN"
#define SOFT_NAME "2023.01.22"

#define DEFAULT_AcftType        1         // [0..15] default aircraft-type: glider
#define DEFAULT_GeoidSepar     40         // [m]
#define DEFAULT_CONbaud    115200         // [bps]
#define DEFAULT_PPSdelay       50         // [ms]
#define DEFAULT_FreqPlan        1

#include "parameters.h"

static FlashParameters Parameters;       // parameters stored in Flash: address, aircraft type, etc.

// ===============================================================================================

static uint16_t BattVoltage = 0;         // [mV] battery voltage, measured every second
static uint8_t  BattCapacity = 0;        // [ %]

static uint8_t calcBattCapacity(uint16_t mVolt)
{ if(mVolt>=4050) return 100;                                 // 4.05V or above => full capacity
  if(mVolt<=3550) return   0;                                 // 3.55V or below => zero capacity
  return (mVolt-3550+2)/5; }                                  // linear dependence (simplified)

// static uint8_t BattCapacity(void) { return BattCapacity(BattVoltage); }

static void ReadBatteryVoltage(void)
{ uint16_t Volt  = getBatteryVoltage();                         // [mv] measure the battery voltage (average over 50 readouts)
  if(BattVoltage==0) BattVoltage = Volt;
  else BattVoltage = (BattVoltage*3+Volt+2)/4;
  BattCapacity = calcBattCapacity(BattVoltage); }

// ===============================================================================================
// GPS: apparently Air530Z is a very different device from Air530, does not look compatible at all

static Air530ZClass GPS;                      // GPS

// details not easy to find: https://www.cnblogs.com/luat/p/14667102.html
// some control NMEA:
// $PCAS01 - serial port baudrate
// $PCAS02 - position update rate
// $PCAS03 - which NMEA's to send out
// $PCAS04 - select navigation system: 1=GPS, 2=BDS, 4=GLS or any sum/or of those
// $PCAS05 - protocol, NMEA version
// $PCAS06 - query info: 0=firmware, 1=hardware, 2=working mode, 3=customer number, 5=upgrade code
// $PCAS10 - restart cold or warm
// $PCAS12 - low power - but does not support this command accord. to the above http link, only the ON_OFF control pin
// $PCAS15 - SBAS control

// GPS.end() simply pulls the power control pin GPIO14 to HIGH/inactive state so the GPS shuts down.
// There is as well the mode-pin of the GPS GPIO1 but it is not clear what it actually does
// ===============================================================================================

#ifdef WITH_BME280                               // this works strictly for BME280, although could possibly be backward compatible to BMP280
static Adafruit_BME280 BME280;                   // pressure/temperature/humidity sensor

static void BME280_Read(GPS_Position &GPS)       // read the pressure/temperature/humidity and put it into the given GPS record
{ float Temp  = BME280.readTemperature();        // [degC]
  GPS.Temperature = floor(Temp*10+0.5);          // [0.1 degC]
  float Press = BME280.readPressure();           // [Pa]
  GPS.Pressure    = floor(Press*4+0.5);          // [1/4 Pa]
  GPS.StdAltitude = Atmosphere::StdAltitude((GPS.Pressure+2)>>2);;
  GPS.hasBaro=1;
  float Hum   = BME280.readHumidity();           // [%]
  GPS.Humidity    = floor(Hum*10+0.5);           // [0.1 %]
  GPS.hasHum=1; }
#endif

#ifdef WITH_BMP280                               // this works strictly for BMP280
static Adafruit_BMP280 BMP280(&Wire);            // pressure/temperature/humidity sensor

static void BME280_Read(GPS_Position &GPS)       // read the pressure/temperature/humidity and put it into the given GPS record
{ float Temp  = BMP280.readTemperature();        // [degC]
  GPS.Temperature = floor(Temp*10+0.5);          // [0.1 degC]
  float Press = BMP280.readPressure();           // [Pa]
  GPS.Pressure    = floor(Press*4+0.5);          // [1/4 Pa]
  GPS.StdAltitude = Atmosphere::StdAltitude((GPS.Pressure+2)>>2);;
  GPS.hasBaro=1; }
#endif

// ===============================================================================================

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

const int RelayQueueSize = 32;
static OGN_PrioQueue<OGN1_Packet, RelayQueueSize> RelayQueue;  // candidate packets to be relayed

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

static int RNG(uint8_t *Data, unsigned Size)
{ while(Size)
  { Random.GPS += micros();
    Random.RX  += analogRead(ADC1);
    XorShift64(Random.Word);
    const uint8_t *Src = (const uint8 *)&Random.Word;
    for(int Idx=0; Idx<8; Idx++)
    { if(Size==0) break;
      *Data++ = Src[Idx];
      Size--; }
  }
  return 1; }

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

static NMEA_RxMsg ConsNMEA;                                     // NMEA catcher for console

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

// static void CONS_CtrlB(void) { Serial.printf("Battery: %5.3fV %d%%\n", 0.001*BattVoltage, BattCapacity); }

static void CONS_CtrlC(void) { PrintParameters(); }

static void CONS_CtrlR(void)
{ uint8_t Len=Format_String(Line, "Relay: ");
  Len+=RelayQueue.Print(Line+Len);
  Len--; Line[Len]=0; Serial.println(Line); }

// const char CtrlB = 'B'-'@';
const char CtrlC = 'C'-'@';
const char CtrlR = 'R'-'@';

static int CONS_Proc(void)
{ int Count=0;
  for( ; ; )
  { uint8_t Byte; int Err=CONS_UART_Read(Byte); if(Err<=0) break;
    Count++;
    // if(Byte==CtrlB) CONS_CtrlB();                                // print battery voltage and capacity -> crashes, why ?!
    if(Byte==CtrlC) CONS_CtrlC();                                // if Ctrl-C received: print parameters
    if(Byte==CtrlR) CONS_CtrlR();                                // print relay queue
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
static  int32_t GPS_Altitude;          // [0.1m]
static  int16_t GPS_GeoidSepar= 0;     // [0.1m]
static uint16_t GPS_LatCosine = 3000;  // [1.0/4096]
static uint8_t  GPS_Satellites = 0;    //

static uint32_t GPS_PPS_ms = 0;                        // [ms] System timer at the most recent PPS
static uint32_t GPS_PPS_UTC = 0;                      // [sec] Unix time which corresponds to the most recent PPS

static uint32_t GPS_Idle=0;                            // [ticks] to detect when GPS stops sending data

static NMEA_RxMsg GpsNMEA;                             // NMEA catcher for GPS

static int GPS_Process(void)                           // process serial data stream from the GPS
{ int Count=0;
  for( ; ; )
  { if(GPS.available()<=0) break;                      // if no more characters then give up
    uint8_t Byte=GPS.read();                           // read character
    // GPS.encode(Byte);                                 // process character through the GPS NMEA interpreter
    GpsNMEA.ProcessByte(Byte);                         // NMEA interpreter
    if(GpsNMEA.isComplete())                           // if NMEA is done
    { if(GpsNMEA.isGxGSV()) ProcessGSV(GpsNMEA);       // process satellite data
      else
      { GPS_Pipe[GPS_Ptr].ReadNMEA(GpsNMEA);           // interpret the position NMEA by the GPS
        // if(GpsNMEA.isGxRMC() || GpsNMEA.isGxGGA() /* || GpsNMEA.isGxGSA() */ )          // selected GPS sentens
        // { GpsNMEA.Data[GpsNMEA.Len]=0; Serial.println((const char *)(GpsNMEA.Data)); } // copy to the console, but this does not work, characters are being lost
      }
      GpsNMEA.Clear(); }
    // Serial.write(Byte);                                // copy character to the console (no loss of characters here)
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
  XorShift64(Random.Word); }

// seems not to work
// static void GPS_HardPPS(void)       // called by hardware interrupt on PPS
// { uint32_t msTime = millis()-GPS_PPS_ms;
//   Serial.printf("PPS: %4d\n", msTime); }

// ===============================================================================================
// OLED pages

static uint8_t OLED_isON=0;

static void OLED_ON(void)
{ if(OLED_isON) return;
  // Serial.println("OLED: ON");
  Display.init(); OLED_isON=1; }

static void OLED_OFF(void)
{ if(!OLED_isON) return;
  // Serial.println("OLED: OFF");
  Display.stop(); OLED_isON=0; }

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

static void OLED_Relay(const GPS_Position &GPS)                 // display list of aircrafts on the Relay list
{
  const char *AcftTypeName[16] = { "----", "Glid", "Tow ", "Heli",
                                   "SkyD", "Drop", "Hang", "Para",
                                   "Pwrd", "Jet ", "UFO", "Ball",
                                   "Zepp", "UAV", "Car ", "Fix " } ;

  Display.clear();
  // Display.setFont(ArialMT_Plain_16);
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  uint8_t VertPos=0;
  for( uint8_t Idx=0; Idx<RelayQueueSize; Idx++)
  { OGN_RxPacket<OGN1_Packet> *Packet = RelayQueue.Packet+Idx; if(Packet->Alloc==0) continue;
    if(Packet->Packet.Header.NonPos) continue;                     // don't show non-position packets
    // int32_t LatDist, LonDist;
    // if(Packet->Packet.calcDistanceVector(LatDist, LonDist, GPS.Latitude, GPS.Longitude, GPS.LatitudeCosine, 20000)<0) continue;
    uint32_t Dist= IntDistance(Packet->LatDist, Packet->LonDist);      // [m]
    uint32_t Dir = IntAtan2(Packet->LonDist, Packet->LatDist);         // [16-bit cyclic]
    Dir &= 0xFFFF; Dir = (Dir*360)>>16;                                // [deg]
    uint8_t Len=0;
    Len+=Format_String(Line+Len, AcftTypeName[Packet->Packet.Position.AcftType]);
    // Line[Len++]=HexDigit(Packet->Packet.Position.AcftType);
    // Line[Len++]=':';
    // Line[Len++]='0'+Packet->Packet.Header.AddrType;                // address-type
    // Line[Len++]=':';
    // Len+=Format_Hex(Line+Len, Packet->Packet.Header.Address, 6);   // address
    Line[Len++]=' ';
    // Len+=Format_SignDec(Line+Len, -(int16_t)Packet->RxRSSI/2);     // [dBm] RSSI
    // Len+=Format_String(Line+Len, "dBm ");
    // Len+=Format_Hex(Line+Len, Packet->Rank);                       // rank for relay
    // Line[Len++]=' ';
    // Line[Len++]=':';
    Len+=Format_UnsDec(Line+Len, Packet->Packet.DecodeAltitude()); // [m] altitude
    Line[Len++]='m'; Line[Len++]=' ';
    Len+=Format_UnsDec(Line+Len, Dir, 3);                             // [deg] direction to target
    Line[Len++]=' ';
    Len+=Format_UnsDec(Line+Len, (Dist+50)/100, 2, 1);                // [km] distance to target
    // Len+=Format_UnsDec(Line+Len, Packet->Packet.Position.Time, 2); // [sec] timestamp
    Len+=Format_String(Line+Len, "km");
    Line[Len]=0;
    Display.drawString(0, VertPos, Line);
    VertPos+=10; if(VertPos>=64) break;
  }
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
    { Len+=Format_UnsDec(Line+Len, BattCapacity); Line[Len++]= '%'; }
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

  Len=0; uint8_t Acfts = RelayQueue.size();
  if(Acfts)
  { Len+=Format_UnsDec(Line+Len, Acfts);
    Len+=Format_String(Line+Len, " Acft");
    Line[Len]=0;
    Display.setTextAlignment(TEXT_ALIGN_LEFT);
    Display.drawString(0, 32, Line); }

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
  if(OLED_CurrPage>=4) OLED_CurrPage=0; }

static void OLED_DispPage(const GPS_Position &GPS)
{ if(!OLED_isON) return;
  switch(OLED_CurrPage)
  { case 2: OLED_Info(); break;
    case 1: OLED_RF(); break;
    case 3: OLED_Relay(GPS); break;
    default: OLED_GPS(GPS); }
}

// ===============================================================================================
// RGB-LED

static void LED_OFF   (void) { Pixels.setPixelColor( 0,   0,   0,   0, 0); Pixels.show(); }
static void LED_Red   (void) { Pixels.setPixelColor( 0, 255,   0,   0, 0); Pixels.show(); }
static void LED_Green (void) { Pixels.setPixelColor( 0,   0,  96,   0, 0); Pixels.show(); }
static void LED_Orange(void) { Pixels.setPixelColor( 0, 255,  48,   0, 0); Pixels.show(); }
static void LED_Yellow(void) { Pixels.setPixelColor( 0, 255,  96,   0, 0); Pixels.show(); }
static void LED_Blue  (void) { Pixels.setPixelColor( 0,   0,   0, 255, 0); Pixels.show(); }
static void LED_Violet(void) { Pixels.setPixelColor( 0,  64,   0, 255, 0); Pixels.show(); }
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

static int getAdslPacket(ADSL_Packet &Packet, const GPS_Position &GPS)  // produce position ADS-L packet
{ Packet.Init();
  Packet.setAddress (Parameters.Address);
  Packet.setAddrType(Parameters.AddrType);
  Packet.setRelay(0);
  Packet.setAcftType(Parameters.AcftType);
  GPS.Encode(Packet);
  return 1; }

static void CleanRelayQueue(uint32_t Time, uint32_t Delay=20) // remove "old" packets from the relay queue
{ uint8_t Sec = (Time-Delay)%60; // Serial.printf("cleanTime(%d)\n", Sec);
  RelayQueue.cleanTime(Sec); }             // remove packets 20(default) seconds into the past

static bool GetRelayPacket(OGN_TxPacket<OGN_Packet> *Packet)      // prepare a packet to be relayed
{ if(RelayQueue.Sum==0) return 0;                     // if no packets in the relay queue
  XorShift64(Random.Word);                             // produce a new random number
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

static LDPC_Decoder      Decoder;      // decoder and error corrector for the OGN Gallager/LDPC code

static void Radio_RxProcess(void)                                      // process packets in the RxFIFO
{ RFM_FSK_RxPktData *RxPkt = RxFIFO.getRead();                         // check for new received packets
  if(RxPkt==0) return;
  LED_Green();                                                         // green flash
  uint8_t RxPacketIdx  = RelayQueue.getNew();                          // get place for this new packet
  OGN_RxPacket<OGN1_Packet> *RxPacket = RelayQueue[RxPacketIdx];
  uint8_t DecErr = RxPkt->Decode(*RxPacket, Decoder);                  // LDPC FEC decoder
  // Serial.printf("%d: Radio_RxProcess( , %d, %d, %d) RxErr=%d/%d %08X { %08X %08X }\n",
  //         millis(), Size, RSSI, SNR, DecErr, RxPacket->RxErr, RxPacket->Packet.HeaderWord, Random.GPS, Random.RX);
  if(DecErr || RxPacket->RxErr>=10 ) { RxFIFO.Read(); LED_OFF(); return; }              // if FEC not correctly decoded or too many bit errors then give up
  uint8_t OwnPacket = ( RxPacket->Packet.Header.Address  == Parameters.Address  )       // is it my own packet (through a relay) ?
                   && ( RxPacket->Packet.Header.AddrType == Parameters.AddrType );
  if(OwnPacket || RxPacket->Packet.Header.NonPos || RxPacket->Packet.Header.Encrypted) { RxFIFO.Read(); LED_OFF(); return; }
  RxPacket->Packet.Dewhiten();
  if(GPS_Satellites)
  { int32_t LatDist=0, LonDist=0;
    bool DistOK = RxPacket->Packet.calcDistanceVector(LatDist, LonDist, GPS_Latitude, GPS_Longitude, GPS_LatCosine)>=0;
    if(DistOK)
    { RxPacket->LatDist = LatDist;
      RxPacket->LonDist = LonDist;
      RxPacket->calcRelayRank(GPS_Altitude/10);                    // calculate the relay-rank (priority for relay)
      // Serial.printf("GPS:%dm Pkt:%dm/%d => Rank:%d\r\n", GPS_Altitude/10, RxPacket->Packet.DecodeAltitude(), RxPacket->RxRSSI, RxPacket->Rank);
      OGN_RxPacket<OGN1_Packet> *PrevRxPacket = RelayQueue.addNew(RxPacketIdx);
      // uint8_t Len=RxPacket->WritePOGNT(Line);
    }
  }
  // Serial.printf("RX[%02X] RSSI:%d, RxErr:%d %08X\n",
  //                 RxPkt->Channel, RxPkt->RSSI, RxPacket->RxErr, RxPacket->Packet.HeaderWord);
  // RxPkt->Print(CONS_UART_Write, 1);
  // uint8_t Len=RxPacket->WritePOGNT(Line);
  // if(Len>=8) { Line[Len-1]=0; Serial.println(Line); }
  RxFIFO.Read();
  LED_OFF(); }

// ===============================================================================================
// Radio

// SYNC sequences include extra zero bytes because the API always asks 8 bytes, even when we ask to use part of the SYNC

// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
static const uint8_t OGN1_SYNC[10] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A, 0x00, 0x00 };

// ADS-L SYNC:       0xF5724B18 encoded in Manchester (fixed packet length 0x18 is included)
static const uint8_t ADSL_SYNC[10] = { 0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65, 0xA9, 0x6A, 0x00, 0x00 };

static uint32_t RF_Slot_UTC = 0;      // [sec] UTC time of the current radio slot
static uint32_t RF_Slot_PPS = 0;      // [ms]  system time of the PPS of the Slot_UTC
static bool     RF_SubSlot  = 0;      // 0 = first TX/RX sub-slot: 0.4-0.8sec 1 = second TX/RX sub-slot: 0.8-1.2sec
static uint8_t  RF_Channel  = 0;      // hopping channel, in Europe/Africa 0 or 1, more channels in Australia, South America and USA/Canada

static int RF_TxTime[2] = { 0, 0 };   // [ms] transmision times for the two sub-slots

static OGN_TxPacket<OGN1_Packet> *RF_TxPkt[2] = { 0, 0 };// OGN packets to be transmitted for the two slots (any can be NULL)
static OGN_TxPacket<OGN1_Packet> *SignTxPkt=0;  // (pointer to) the packet which is digitally signed

static uint8_t RF_TxPktData[2][2*26];          // packet data to be transmitted, after Manchester encoding

static OGN_TxPacket<OGN1_Packet> *ADSL_TxPkt=0; // which OGN packet is the ADS-L packet linked to
static bool ADSL_TxSlot=0;                      // during which slot to transmit the ADS-L packet

static uint16_t RF_RxPackets = 0;            // [packets] counts received packets
static uint16_t RF_TxPackets = 0;

static RadioEvents_t Radio_Events;

static int SubSlotStart(bool SubSlot) { return SubSlot?800:350; }

// [ms] time left for the receiver to listen for packets
static int RxTime(int msTime, int msTxTime=(-1))
{ int msStart = SubSlotStart(RF_SubSlot);   // [ms] sub-slot start, relative to RF_Slot_PPS
  int msEnd = msStart+450;                  // [ms] time to stop listenning
  if(msTxTime>0) msEnd = msStart+msTxTime;  // [ms] time to stop listenning in case there is a packet to transmit
  msTime = msEnd-msTime;                    // [ms] time left
  if(msTime<0) msTime=0;                    //
  else if(msTime>550) msTime=550;
  return msTime; }                          // [ms] time remaining till the end-of-sub-slot
                                            // if zero then the sub-slot is over

static void Radio_Receive(int msTime)
{ // Radio.Rx(msWait);
  Radio.RxBoosted(msTime);                  // higher sensitivity reception but 2mA more current
  // CY_PM_WFI;                              // go to sleep and wake up on an interrupt
}

static void Radio_TxDone(void)              // end-of-transmission interrupt
{ RF_TxPackets++;
  uint32_t msTime = millis()-RF_Slot_PPS;   // [ms] relative to the PPS of this RF slot
  int msWait=RxTime(msTime);
  // Serial.printf("TxDone: %4d:%3d\n", msTime, msWait);
  OGN_RxConfig();
  if(msWait>5) Radio_Receive(msWait);
          else Radio_NewSubSlot(); }         //

static void Radio_TxTimeout(void)
{ uint32_t msTime = millis()-RF_Slot_PPS;    // [ms] relative to PPS
  int msWait=RxTime(msTime);
  // Serial.printf("TxTout: %4d:%3d\n", msTime, msWait);
  OGN_RxConfig();
  if(msWait>5) Radio_Receive(msWait);
          else Radio_NewSubSlot(); }

static void Radio_NewSlot(void)
{ RF_Slot_UTC=GPS_PPS_UTC;
  RF_Slot_PPS=GPS_PPS_ms;
  XorShift64(Random.Word);
  RF_TxTime[0]=0; RF_TxTime[1]=0;
  if(RF_TxPkt[0])                                     // copy Slot0 packet to be transmitted
  { Manchester(RF_TxPktData[0], RF_TxPkt[0]->Packet.Byte(), RF_TxPkt[0]->Bytes);
    RF_TxTime[0] = Random.RX  % 389 + 50;             // randomize transmit times within slots
    RF_TxPkt[0] = 0; }
  if(RF_TxPkt[1])                                     // copy Slot1 packet to be transmitted
  { Manchester(RF_TxPktData[1], RF_TxPkt[1]->Packet.Byte(), RF_TxPkt[1]->Bytes);
    RF_TxTime[1] = Random.GPS % 389 +  5;
    RF_TxPkt[1] = 0; }
  // printf("RFslot: %d %3d:%3d\n", RF_Slot_UTC, RF_TxTime[0], RF_TxTime[1]);
  RF_SubSlot=0; }

static void Radio_NewSubSlot(void)
{ OGN_TxConfig();                                     // renew the configuration for Tx/Rx just in case
  OGN_RxConfig();
  uint32_t msTime = millis()-RF_Slot_PPS;
  if(msTime>=1250 || RF_SubSlot) Radio_NewSlot();                // in case a new slot
                            else RF_SubSlot=1;
  msTime = millis()-RF_Slot_PPS;
  RF_Channel=Radio_FreqPlan.getChannel(RF_Slot_UTC, RF_SubSlot, 1); // calc. the channel for this (sub)slot
  Radio.SetChannel(Radio_FreqPlan.getChanFrequency(RF_Channel));    // set the radio to the channel frequency
  int msWait=RxTime(msTime, RF_TxTime[RF_SubSlot]);                 // time to listen (before possible transmission)
  // printf("RFsubSlot: %4d:%3d S%d C%d\n", msTime, msWait, RF_SubSlot, RF_Channel);
  if(msWait<=0) msWait=450;                          // should never happen but just in case
  Radio_Receive(msWait); }                           // go to receive mode

static void Radio_RxTimeout(void)                  // end-of-receive-period interrupt
{ uint32_t msTime = millis()-RF_Slot_PPS;
  int msWait=RxTime(msTime, RF_TxTime[RF_SubSlot]);
  // Serial.printf("RxTout: %4d:%3d S%d C%d\n", msTime, msWait, RF_SubSlot, RF_Channel);
  if(msWait<=0 && RF_TxTime[RF_SubSlot])
  { Radio.Send(RF_TxPktData[RF_SubSlot], 2*26); RF_TxTime[RF_SubSlot]=0; /* CY_PM_WFI; */ }
  else Radio_NewSubSlot(); }

// a new packet has been received callback - this should probably be a quick call
static void Radio_RxDone( uint8_t *Packet, uint16_t Size, int16_t RSSI, int8_t SNR) // RSSI and SNR are not passed for FSK packets
{ RF_RxPackets++;
  uint32_t msTime = millis()-RF_Slot_PPS;                              // [ms] relative to PPS
  int msWait=RxTime(msTime, RF_TxTime[RF_SubSlot]);
  // Serial.printf("RxDone: %4d:%3d S%d C%d\n", msTime, msWait, RF_SubSlot, RF_Channel);
  if(Size!=2*26) { /* CY_PM_WFI; */ return; }                                // for now, only treat OGN packets
  PacketStatus_t RadioPktStatus; // to get the packet RSSI: https://github.com/HelTecAutomation/CubeCell-Arduino/issues/236
  SX126xGetPacketStatus(&RadioPktStatus);
  RSSI = RadioPktStatus.Params.Gfsk.RssiAvg;                           // [dBm] RSSI
  RFM_FSK_RxPktData *RxPkt = RxFIFO.getWrite();                        // new packet in the RxFIFO
  RxPkt->Time = RF_Slot_UTC;                                           // [sec]
  RxPkt->msTime = msTime;                                              // [ms] time since PPS
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
  Random.RX = (Random.RX*RSSI) ^ (~RSSI); XorShift64(Random.Word);     // update random number
  if(msWait>0) Radio_Receive(msWait);
  else if(RF_TxTime[RF_SubSlot])
  { Radio.Send(RF_TxPktData[RF_SubSlot], 2*26); RF_TxTime[RF_SubSlot]=0; /* CY_PM_WFI; */ }
  else Radio_NewSubSlot(); }

extern SX126x_t SX126x; // access to LoraWan102 driver parameters in LoraWan102/src/radio/radio.c

// additional RF configuration reuired for OGN/ADS-L to work
static void Radio_UpdateConfig(const uint8_t *SyncWord, uint8_t SyncBytes)
{ SX126x.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_05;
  SX126xSetModulationParams(&SX126x.ModulationParams);
  SX126x.PacketParams.Params.Gfsk.SyncWordLength = SyncBytes*8;
  SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREE_OFF;
  SX126xSetPacketParams(&SX126x.PacketParams);
  SX126xSetSyncWord((uint8_t *)SyncWord); }

static void OGN_TxConfig(void)
{ Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower, 50000, 0, 100000, 0, 1, 1, 0, 0, 0, 0, 20);
  // Modem, TxPower, Freq-dev [Hz], LoRa bandwidth, Bitrate [bps], LoRa code-rate, preamble [bytes],
  // Fixed-len [bool], CRC-on [bool], LoRa freq-hop [bool], LoRa hop-period [symbols], LoRa IQ-invert [bool], Timeout [ms]
  Radio_UpdateConfig(OGN1_SYNC, 8); }

static void ADSL_TxConfig(void)
{ Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower, 50000, 0, 100000, 0, 1, 1, 0, 0, 0, 0, 20);
  Radio_UpdateConfig(ADSL_SYNC, 8); }

static void OGN_RxConfig(void)
{ Radio.SetRxConfig(MODEM_FSK, 200000, 100000, 0, 200000, 1, 100, 1, 52, 0, 0, 0, 0, true);
  // Modem, Bandwidth [Hz], Bitrate [bps], CodeRate, AFC bandwidth [Hz], preamble [bytes], Timeout [bytes], FixedLen [bool], PayloadLen [bytes], CRC [bool],
  // FreqHopOn [bool], HopPeriod, IQinvert, rxContinous [bool]
  Radio_UpdateConfig(OGN1_SYNC+1, 7); }

static void ADSL_RxConfig(void)
{ Radio.SetRxConfig(MODEM_FSK, 200000, 100000, 0, 200000, 1, 100, 1, 48, 0, 0, 0, 0, true); // same as OGN just different packet size
  Radio_UpdateConfig(ADSL_SYNC+1, 7); }                                                       // and different SYNC

// Manchester encode packet data
static int Manchester(uint8_t *TxData, const uint8_t *PktData, int PktLen)
{ int TxLen=0;
  for(int Idx=0; Idx<PktLen; Idx++)
  { uint8_t Byte=PktData[Idx];
    TxData[TxLen++]=ManchesterEncode[Byte>>4];                      // software manchester encode every byte
    TxData[TxLen++]=ManchesterEncode[Byte&0x0F]; }
  return TxLen; }

// transmit Data with Manchester encoding optionally followed by a signature (without Manchester), setup already assumed done
static int Transmit(const uint8_t *Data, uint8_t PktLen=26, const uint8_t *Sign=0, uint8_t SignLen=68)
{ static uint8_t TxPacket[2*26+64+4];                                  // buffer to fit manchester encoded packet and the digital signature
  int TxLen = Manchester(TxPacket, Data, PktLen);
  if(SignLen && Sign)
  { for(uint8_t Idx=0; Idx<SignLen; Idx++)                            // digital signature
    { uint8_t Byte=Sign[Idx];
      TxPacket[TxLen++]=Byte; }                                       // copy the bytes directly, without Manchester encoding
  }
  Radio.Send(TxPacket, TxLen);
  return TxLen; }

// transmit OGN packet with possible signature
static int OGN_Transmit(const OGN_TxPacket<OGN1_Packet> &TxPacket, uint8_t *Sign=0, uint8_t SignLen=68)
{ OGN_TxConfig();
  return Transmit(TxPacket.Byte(), TxPacket.Bytes, Sign, SignLen); }

// transmit ADS-L packet with possible signature
static int ADSL_Transmit(const ADSL_Packet &TxPacket, uint8_t *Sign=0, uint8_t SignLen=68) // transmit an ADS-L packet
{ ADSL_TxConfig();
  return Transmit(&(TxPacket.Version), TxPacket.TxBytes-3, Sign, SignLen); }

// ===============================================================================================
// User button and Power-OFF

static void Sleep(void)                            // shut-down all hardware and go to deep sleep
{ detachInterrupt(USER_KEY);                       // stop user-button interrupt
  // detachInterrupt(GPIO11);                         // stop GPS PPS interrupt
  GPS.end();                                       // stop GPS
  detachInterrupt(RADIO_DIO_1);                    // stop Radio interrupt
  Radio.Sleep();                                   // stop Radio
  OLED_OFF();                                      // stop OLED
  LED_OFF();                                       // stop RGB LED
  Wire.end();                                      // stop I2C
  Serial.end();                                    // stop console UART
  pinMode(Vext, ANALOG);
  pinMode(ADC, ANALOG);
#ifdef WITH_DEBUGPIN
  pinMode(DebugPin, ANALOG);
#endif
  while(1) lowPowerHandler(); }                     // never wake up

static bool Button_isPressed(void) { return digitalRead(USER_KEY)==0; } // tell if button is being pressed or not at this moment

static bool     Button_LowPower=0;                  // set to one when button pressed for more than one second.

static uint32_t Button_PressTime = 0;
static uint8_t Button_ShortPush = 0;
static uint32_t Button_PrevSysTime=0;               // [ms] previous sys-time when the Button_Process() was called
static uint32_t Button_IdleTime=0;                  // [ms] count time when the user is not pushing the button

static void Button_Process(void)                    // process the button push/release: as press-release works on interrupts, this can be now called less often
{ uint32_t SysTime = millis();                      // [ms]
  if(Button_isPressed())
  { uint32_t LongPush=SysTime-Button_PressTime;
    if(LongPush>1000) Button_LowPower=1; }
  uint32_t Diff = SysTime-Button_PrevSysTime;
  if(Button_ShortPush==0)
  { Button_IdleTime+=Diff;                          // count idle time
    if(Button_IdleTime>60000) Button_IdleTime=60000; // [ms] top it at 60 sec
    if(Button_IdleTime>=20000) OLED_OFF(); }        // turn off OLED when idle more than 20sec
  while(Button_ShortPush)
  { if(OLED_isON) OLED_NextPage();                  // either switch OLED page
             else OLED_ON();                        // or turn the OLED back ON
    Button_IdleTime=0;
    Button_ShortPush--; }
  Button_PrevSysTime=SysTime; }

static void Button_ChangeInt(void)  // called by hardware interrupt on button push or release
{ if(Button_isPressed())            // pressed
  { Button_PressTime=millis(); }
  else                              // released
  { uint32_t Diff = millis()-Button_PressTime;
    if(Diff<100)                    // it appears something is "pushing" the button at 1sec interval for 12-13ms - could it be battery readout ?
    { // Serial.printf("ButtPulse: %d\n", Diff);
      return; }
    if(Diff<400) Button_ShortPush++;
  }
}

// ===============================================================================================

void setup()
{ // delay(2000); // prevents USB driver crash on startup, do not omit this

  Random.Word ^= getUniqueID();

  VextON();                               // Turn on power to OLED (and possibly other external devices)
  delay(100);
#ifdef WITH_DEBUGPIN
  DebugPin_Init();
#endif
  ReadBatteryVoltage();

  Parameters.ReadFromFlash();             // read parameters from Flash
#ifdef HARD_NAME
  strcpy(Parameters.Hard, HARD_NAME);
#endif
#ifdef SOFT_NAME
  strcpy(Parameters.Soft, SOFT_NAME);
#endif
  // Parameters.WriteToFlash();

  Serial.begin(Parameters.CONbaud);       // Start console/debug UART
  // Serial.setRxBufferSize(120);            // this call has possibly no effect and buffer size is always 255 bytes
  // Serial.setTxBufferSize(512);            // this call does not even exist and buffer size is not known
  // while (!Serial) { }                  // wait for USB serial port to connect

  Serial.println("OGN Tracker on HELTEC CubeCell with GPS");

  pinMode(USER_KEY, INPUT_PULLUP);        // push button
  attachInterrupt(USER_KEY, Button_ChangeInt, CHANGE);

  Pixels.begin();                         // Start RGB LED
  Pixels.clear();
  Pixels.setBrightness(0x20);
  LED_Green();
  // Pixels.setPixelColor( 0, 255, 0, 0, 0); // Green
  // Pixels.show();

  OLED_ON();
  // Display.init();                                     // Start the OLED
  // OLED_isON=1;
  OLED_Logo();

#ifdef WITH_BME280
  for(uint8_t Addr=0x76; Addr<=0x77; Addr++)
  { if(BME280.begin(0x76, &Wire)) break;                        // BME280 on the I2C
    Serial.printf("BME280 not detected at 0x%02X\n", Addr); }
#endif
#ifdef WITH_BMP280
  for(uint8_t Addr=0x76; Addr<=0x77; Addr++)
  { if(BMP280.begin(0x76)) break;                               // BMP280 on the I2C
    Serial.printf("BMP280 not detected at 0x%02X\n", Addr); }
#endif

  GPS.begin(115200);                                 // Start the GPS
  // pinMode(GPIO11, INPUT);                            // GPS PPS ?
  // attachInterrupt(GPIO11, GPS_HardPPS, RISING);

  // Serial.println("GPS started");
  Radio_FreqPlan.setPlan(Parameters.FreqPlan);       // set the frequency plan from the parameters
  Radio_Events.TxDone    = Radio_TxDone;             // Start the Radio
  Radio_Events.TxTimeout = Radio_TxTimeout;
  Radio_Events.RxTimeout = Radio_RxTimeout;
  Radio_Events.RxDone    = Radio_RxDone;
  Radio.Init(&Radio_Events);
  Radio.SetChannel(Radio_FreqPlan.getFrequency(0));
  OGN_TxConfig();
  OGN_RxConfig();
  Radio.Rx(2000);
  // Serial.println("Radio started\n");
  Random.RX  ^= Radio.Random();
  Random.GPS ^= Radio.Random();
  XorShift64(Random.Word);
  RX_RSSI.Set(-2*110);

#ifdef WITH_DIG_SIGN
  uECC_set_rng(&RNG);
  SignKey.Init();
  SignKey.PrintKeys();
#endif

}

static OGN_TxPacket<OGN1_Packet> TxPosPacket, TxStatPacket, TxRelPacket, TxInfoPacket; // OGN position, status and info packets
static ADSL_Packet ADSL_TxPosPacket;           // ADS-L position packet

// static uint8_t OGN_Sign[68];                // digital signature to be appended to some position packets
// static uint8_t OGN_SignLen=0;               // digital signature size, 64 + 1 or 2 bytes

static bool GPS_Done = 0;                   // State: 1 = GPS is sending data, 0 = GPS sent all data, waiting for the next PPS

static int32_t  RxRssiSum=0;                // sum RSSI readouts
static int      RxRssiCount=0;              // count RSSI readouts

static void EndOfGPS(void)                                 // start the TX/RX time slot after the GPS completes sending data
{ // Serial.printf("EoGPS [%d]\n", RxRssiCount);
  if(RxRssiCount)
  { RX_RSSI.Process(RxRssiSum/RxRssiCount); RxRssiSum=0; RxRssiCount=0; }
  XorShift64(Random.Word);
  GPS_Position &GPS = GPS_Pipe[GPS_Ptr];
  GPS_Satellites = GPS.Satellites;
  if(GPS.isTimeValid() && GPS.isDateValid()) { LED_Blue(); GPS_PPS_UTC = GPS.getUnixTime(); }    // if time and date are valid
                                       else  { LED_Yellow(); }
  RX_OGN_Count64 += RF_RxPackets - RX_OGN_CountDelay.Input(RF_RxPackets); // add OGN packets received, subtract packets received 64 seconds ago
  RF_RxPackets=0;                                                       // clear the received packet count
  CleanRelayQueue(GPS_PPS_UTC);
  bool TxPos=0;
  if(GPS.isValid())                                           // if position is valid
  { GPS_Altitude  = GPS.Altitude;                             // set global GPS variables
    GPS_Latitude  = GPS.Latitude;
    GPS_Longitude = GPS.Longitude;
    GPS_GeoidSepar= GPS.GeoidSeparation;
    GPS_LatCosine = GPS.LatitudeCosine;
    Radio_FreqPlan.setPlan(GPS_Latitude, GPS_Longitude);      // set Radio frequency plan
    GPS_Random_Update(GPS);
    XorShift64(Random.Word);
    getPosPacket(TxPosPacket.Packet, GPS);                    // produce position packet to be transmitted
#ifdef WITH_DIG_SIGN
    if(SignKey.KeysReady)
    { SignKey.Hash(GPS_PPS_UTC, TxPosPacket.Packet.Byte(), TxPosPacket.Packet.Bytes); // produce SHA256 hash (takes below 1ms)
      SignTxPkt = &TxPosPacket; }
#endif
    TxPosPacket.Packet.Whiten();
    TxPosPacket.calcFEC();                                    // position packet is ready for transmission
#ifdef WITH_ADSL
    ADSL_TxPkt = &TxPosPacket;
    getAdslPacket(ADSL_TxPosPacket, GPS);
    ADSL_TxPosPacket.Scramble();                              // this call hangs when -Os is used to compile
    ADSL_TxPosPacket.setCRC();
    ADSL_TxSlot = Random.GPS&0x20;
#endif
    TxPos=1; }
  CONS_Proc();
  ReadBatteryVoltage();
  CONS_Proc();
  OLED_DispPage(GPS);                                         // display GPS data or other page on the OLED
  CONS_Proc();
  RF_TxPkt[0]=RF_TxPkt[1]=0;
  if(TxPos) RF_TxPkt[0] = RF_TxPkt[1] = &TxPosPacket;
  static uint8_t InfoTxBackOff=0;
  static uint8_t InfoToggle=0;
  if(InfoTxBackOff) InfoTxBackOff--;
  else
  { InfoToggle = !InfoToggle;
    int Ret=0;
    if(InfoToggle) Ret=getInfoPacket(TxInfoPacket.Packet);      // try to get the next info field
    if(Ret<=0) Ret=getStatusPacket(TxInfoPacket.Packet, GPS);   // if not any then prepare a status packet
    if(Ret>0)
    { TxInfoPacket.Packet.Whiten(); TxInfoPacket.calcFEC();     // prepare the packet for transmission
      if(Random.RX&0x10) RF_TxPkt[1] = &TxInfoPacket;                // put it randomly into 1st or 2nd time slot
                    else RF_TxPkt[0] = &TxInfoPacket;
      InfoTxBackOff = 15 + (Random.RX%3);
    }
  }
  XorShift64(Random.Word);
  static uint8_t RelayTxBackOff=0;
  if(RelayTxBackOff) RelayTxBackOff--;
  else if(GetRelayPacket(&TxRelPacket))
  { if(Random.RX&0x20) RF_TxPkt[1] = &TxRelPacket;
                  else RF_TxPkt[0] = &TxRelPacket;
    RelayTxBackOff = Random.RX%3; }
  GPS_Next();
  XorShift64(Random.Word);
#ifdef WITH_DIG_SIGN
  static uint8_t SignTxBackOff=0;
  // Serial.printf("SignTx: %2ds %d:%d:%d\n", SignTxBackOff, TxPos, SignKey.KeysReady, SignKey.HashReady);
  if(SignTxBackOff) SignTxBackOff--;
  else
  { if(TxPos && SignKey.KeysReady && SignKey.HashReady)
    { LED_Violet(); // Serial.printf("Sign: calc. start\n");
      // SignKey.PrintHash();
      SignKey.Sign();                                              // calc. the signature
      // SignKey.PrintSign();
      // Serial.printf("Sign: calc. stop\n");
      // RF_TxTime1 = 200 + (RF_TxTime0/2);
      SignTxBackOff = 6 + (Random.RX%7); }
  }
#endif
  LED_OFF();
}

void loop()
{

#ifdef WITH_DEBUGPIN
  DebugPin_ON(1);                                                 // for debug: to watch the sleep time on the scope
#endif

  CY_PM_WFI; //  __WFI();                                         // sleep and wake up on an interrupt

#ifdef WITH_DEBUGPIN
  DebugPin_ON(0);
#endif

  Button_Process();                                               // check for button short/long press
  if(Button_LowPower) { Sleep(); return; }                        // enter deep sleep when power-off requested

  Radio_RxProcess();                                              // process received packets, if any

  CONS_Proc();                                                    // process input from the console
  if(GPS_Process()==0) { GPS_Idle++; /* delay(1); */ }                  // process input from the GPS
                  else { GPS_Idle=0; /* RxRssiSum+=2*Radio.Rssi(MODEM_FSK); RxRssiCount++; */ } // [0.5dBm]
  if(GPS_Done)                                                    // if state is GPS not sending data
  { if(GPS_Idle<2)                                                // GPS (re)started sending data
    { GPS_Done=0;                                                 // change the state to GPS is sending data
      GPS_PPS_ms = millis() - Parameters.PPSdelay;                // [ms] record the est. PPS time
      GPS_PPS_UTC++; }                                            // [sec]
  }
  else
  { if(GPS_Idle>10)                                               // GPS stopped sending data
    { // printf("GPSstop: %ds\n", GPS_PPS_UTC);
      EndOfGPS();                                              // start the RF slot
      GPS_Done=1; }
  }

}
