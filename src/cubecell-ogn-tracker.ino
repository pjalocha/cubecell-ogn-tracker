#include "Arduino.h"
#include <Wire.h>  

#include "LoRaWan_APP.h"

#include "GPS_Air530.h"
#include "GPS_Air530Z.h"

#include "HT_SSD1306Wire.h"
#include "CubeCell_NeoPixel.h"

#include "format.h"
#include "nmea.h"
#include "manchester.h"
#include "ogn1.h"

#include "rfm.h"

static uint64_t getUniqueID(void) { return getID(); }        // get unique serial ID of the CPU/chip
static uint32_t getUniqueAddress(void) { return getID()&0x00FFFFFF; }

#define WITH_OGN1                          // OGN protocol version 1/2
#define OGN_Packet OGN1_Packet

#define HARDWARE_ID 0x03
#define SOFTWARE_ID 0x01

#define DEFAULT_AcftType        1         // [0..15] default aircraft-type: glider
#define DEFAULT_GeoidSepar     40         // [m]
#define DEFAULT_CONbaud    115200         // [bps]
#define DEFAULT_PPSdelay       80         // [ms]
#define DEFAULT_FreqPlan        1

#include "parameters.h"

static FlashParameters Parameters;                                            // parameters address, type, etc.

// static Air530Class GPS;                                                       // GPS
static Air530ZClass GPS;

static SSD1306Wire Display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10 ); // OLED: addr , freq , i2c group , resolution , rst

static CubeCell_NeoPixel Pixels(1, RGB, NEO_GRB + NEO_KHZ800);                // three-color LED

static char Line[256];                                                        // for printing

void VextON(void)                   // Vext controls OLED power
{ pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW); }

void VextOFF(void)                  // Vext default OFF
{ pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH); }

/*
int GPS_UART_Read(uint8_t &Byte)  // read byte from the GPS serial port
{ int Ret=GPS.read();
  if(Ret>=0) Byte=Ret;
  return Ret; }

void GPS_UART_Write(char Byte) // write byte to the GPS serial port
{ GPS.write(Byte); }

void  GPS_UART_SetBaudrate(int BaudRate) { GPS.begin(BaudRate); }
*/

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

static void ConsNMEA_Process(void)
{ if(!ConsNMEA.isPOGNS()) return;
  if(ConsNMEA.hasCheck() && !ConsNMEA.isChecked() ) return;
  if(ConsNMEA.Parms==0) { PrintPOGNS(); return; }
  Parameters.ReadPOGNS(ConsNMEA);
  PrintParameters();
  Parameters.WriteToFlash();
}

static void CONS_CtrlC(void)
{ PrintParameters(); }

static int CONS_Proc(void)
{ int Count=0;
  for( ; ; )
  { uint8_t Byte; int Err=CONS_UART_Read(Byte); if(Err<=0) break;
    Count++;
    if(Byte==0x03) CONS_CtrlC();                                // if Ctrl-C received: print parameters
    ConsNMEA.ProcessByte(Byte);
    if(ConsNMEA.isComplete())
    { ConsNMEA_Process();                                           // interpret the ConsNMEA
      ConsNMEA.Clear(); }
  }
  return Count; }

// ===============================================================================================
// Process GPS data

static NMEA_RxMsg GpsNMEA;

static int          GPS_Ptr = 0;
static GPS_Position GPS_Pipe[2];

static uint32_t GPS_Idle=0;

static int GPS_Process(void)                           // process serial data stream from the GPS
{ int Count=0;
  for( ; ; )
  { if(GPS.available()<=0) break;                      // if no more characters then give up
    uint8_t Byte=GPS.read();                           // read character
    // GPS.encode(Byte);                                 // process character through the GPS NMEA interpreter
    GpsNMEA.ProcessByte(Byte);                         // NMEA interpreter
    if(GpsNMEA.isComplete())                           // if NMEA is done
    { GPS_Pipe[GPS_Ptr].ReadNMEA(GpsNMEA);             // interpret the NMEA by the GPS
      GpsNMEA.Clear(); }
    Serial.write(Byte);                                // copy character to the console (we could copy only the selected and correct sentences)
    Count++; }                                         // count processed characters
  return Count; }                                      // return number of processed characters

static void GPS_Next(void)
{ int Next = GPS_Ptr^1;
  if(GPS_Pipe[GPS_Ptr].isValid() && GPS_Pipe[Next].isValid()) GPS_Pipe[GPS_Ptr].calcDifferentials(GPS_Pipe[Next]);
  GPS_Ptr=Next; }

static void OLED_GPS(void)                             // display time, date and GPS data/status
{ Display.clear();
  // Display.setFont(ArialMT_Plain_10);
  Display.setFont(ArialMT_Plain_16);
  const GPS_Position &GPS = GPS_Pipe[GPS_Ptr];
  char Line[16];
  if(GPS.isTimeValid())
  { int Idx = sprintf(Line, "%02d:%02d:%02d", GPS.Hour, GPS.Min, GPS.Sec);
    Line[Idx] = 0; }
  else { strcpy(Line, "--:--:--"); }
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.drawString(128, 0, Line);
  if(GPS.isTimeValid() && GPS.isDateValid())
  { int Idx = sprintf(Line, "%02d.%02d.%02d", GPS.Day, GPS.Month, GPS.Year);
    Line[Idx] = 0; }
  else { strcpy(Line, "--.--.--"); }
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 0, Line);
  if(GPS.isValid())
  { }
  Display.display(); }

// ===============================================================================================
// RGB-LED

static void LED_OFF   (void) { Pixels.setPixelColor( 0,   0,   0,   0, 0); Pixels.show(); }
static void LED_Red   (void) { Pixels.setPixelColor( 0, 255,   0,   0, 0); Pixels.show(); }
static void LED_Orange(void) { Pixels.setPixelColor( 0, 255,  64,   0, 0); Pixels.show(); }
static void LED_Green (void) { Pixels.setPixelColor( 0,   0, 255,   0, 0); Pixels.show(); }
static void LED_Yellow(void) { Pixels.setPixelColor( 0, 255, 255,   0, 0); Pixels.show(); }
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

static int getPosPacket(OGN1_Packet &Packet)                           // produce position OGN packet
{ Packet.HeaderWord = 0;
  Packet.Header.Address = Parameters.Address;                          // aircraft address
  Packet.Header.AddrType = Parameters.AddrType;                        // aircraft address-type
  Packet.calcAddrParity();
  Packet.Position.AcftType = Parameters.AcftType;                      // aircraft type
  //
  return 0; }

// ===============================================================================================
// Radio

// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
static const uint8_t OGN1_SYNC[10] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A, 0x00, 0x00 };
// OGNv2 SYNC:       0xF56D3738 encoded in Machester
static const uint8_t OGN2_SYNC[10] = { 0x55, 0x99, 0x96, 0x59, 0xA5, 0x95, 0xA5, 0x6A, 0x00, 0x00 };

static const uint8_t PAW_SYNC [10] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x18, 0x71, 0x00, 0x00 };

static RadioEvents_t Radio_Events;

static void Radio_TxDone(void)
{ // Serial.printf("%d: Radio_TxDone()\n", millis());
  Radio.Rx(0); }

static void Radio_TxTimeout(void)
{ // Serial.printf("%d: Radio_TxTimeout()\n", millis());
  Radio.Rx(0); }

static LDPC_Decoder      Decoder;      // decoder and error corrector for the OGN Gallager/LDPC code
static RFM_FSK_RxPktData RxPktData;
static OGN_RxPacket<OGN1_Packet> RxPacket;

static void Radio_RxDone( uint8_t *Packet, uint16_t Size, int16_t RSSI, int8_t SNR)
{ if(Size!=2*26) return;
  uint8_t PktIdx=0;
  for(uint8_t Idx=0; Idx<26; Idx++)                                     // loop over packet bytes
  { uint8_t ByteH = Packet[PktIdx++];
    ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode manchester, detect (some) errors
    uint8_t ByteL = Packet[PktIdx++];
    ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F;
    RxPktData.Data[Idx]=(ByteH<<4) | ByteL;
    RxPktData.Err [Idx]=(ErrH <<4) | ErrL ;
  }
  RxPktData.Time = 0;
  RxPktData.msTime = 0;
  RxPktData.Channel = 0;
  RxPktData.RSSI = -2*RSSI;
  if(RxPktData.Decode(RxPacket, Decoder)==0 && RxPacket.RxErr<10)    // if FEC correctly decoded
  {
  }
  Serial.printf("%d: Radio_RxDone( , %d, %d, %d) RxErr=%d %08X\n", millis(), Size, RSSI, SNR, RxPacket.RxErr, RxPacket.Packet.HeaderWord);
  // RxPktData.Print(CONS_UART_Write, 1);
}

static void OGN_TxConfig(void)
{ Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower, 50000, 0, 100000, 0, 1, 1, 0, 0, 0, 0, 20, 8, OGN1_SYNC); }

static void PAW_TxConfig(void)      // incorrect because the PAW preamble needs to be long and the library does not support it
{ Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower+6, 9600, 0, 38400, 0, 1, 1, 0, 0, 0, 0, 20, 8, PAW_SYNC); }

static void OGN_RxConfig(void)
{ Radio.SetRxConfig(MODEM_FSK, 250000, 100000, 0, 250000, 1, 100, 1, 52, 0, 0, 0, 0, true, 8, OGN1_SYNC); }

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

static bool Button_isPressed(void) { return digitalRead(USER_KEY)==0; }

static void Button_Process(void)
{ 
}

// ===============================================================================================

void setup()
{ // delay(2000); // prevents USB driver crash on startup, do not omit this

  VextON();                               // Turn on power to OLED (and possibly other external devices)
  delay(100);

  Serial.begin(115200);                   // Start console/debug UART
  // while (!Serial) { }                  // wait for USB serial port to connect
  Serial.println("OGN Tracker on HELTEC CubeCell");

  pinMode(USER_KEY, INPUT);               // push button

  Pixels.begin();                         // Start RGB LED
  Pixels.clear();
  Pixels.setBrightness(0x08);
  LED_Green();
  // Pixels.setPixelColor( 0, 255, 0, 0, 0); // Green
  // Pixels.show();

  Parameters.ReadFromFlash();             // read parameters from Flash

  Display.init();                         // Start the OLED
  Display.clear();
  Display.display();
  Display.setTextAlignment(TEXT_ALIGN_CENTER);      // 
  Display.setFont(ArialMT_Plain_16);
  Display.drawString(64, 32-16/2, "OGN-Tracker");
  Display.display();

  GPS.begin(38400);                                  // Start the GPS

  Radio_Events.TxDone    = Radio_TxDone;             // Start the Radio
  Radio_Events.TxTimeout = Radio_TxTimeout;
  Radio_Events.RxDone    = Radio_RxDone;
  Radio.Init(&Radio_Events);
  Radio.SetChannel(868400000);
  OGN_TxConfig();
  OGN_RxConfig();

}

static OGN_TxPacket<OGN1_Packet> TxPacket;

static uint32_t GPS_Start = 0; // [ms]

static bool GPS_Done = 0;      // State: 1 = GPS is sending data, 0 = GPS sent all data, waiting for the next PPS

void loop()
{
  CONS_Proc();
  Button_Process();
  if(GPS_Process()==0) { GPS_Idle++; delay(1); }
                  else { GPS_Idle=0; }
  if(GPS_Done)
  { if(GPS_Idle<3) { GPS_Done=0; GPS_Start=millis(); LED_Blue(); } } // GPS started sending data
  else
  { if(GPS_Idle>5)                                                // GPS stopped sending data
    { if(GPS.date.isValid()) LED_Green();                         //
                       else  LED_Yellow();
      // here we start the slot just after the GPS sent its data
      // printf("%d: Batt\n", millis());
      uint16_t BattVolt = getBatteryVoltage();                    // read the battery voltage [mV]
      // printf("BattVolt=%d mV\n", BattVolt);
      // printf("%d: OLED\n", millis());
      OLED_GPS();                                                 // display GPS data on the OLED
      if(getInfoPacket(TxPacket.Packet))
      { TxPacket.Packet.Whiten(); TxPacket.calcFEC();
        // printf("%d: Radio\n", millis());
        OGN_Transmit(TxPacket); }
      // printf("%d:\n", millis());
      LED_OFF();
      GPS_Next();
      GPS_Done=1; }
  }


}
