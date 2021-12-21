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

static NMEA_RxMsg NMEA;

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

static void NMEA_Process(void)
{ if(!NMEA.isPOGNS()) return;
  if(NMEA.hasCheck() && !NMEA.isChecked() ) return;
  if(NMEA.Parms==0) { PrintPOGNS(); return; }
  Parameters.ReadPOGNS(NMEA);
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
    NMEA.ProcessByte(Byte);
    if(NMEA.isComplete())
    { NMEA_Process();                                           // interpret the NMEA
      NMEA.Clear(); }
  }
  return Count; }

// ===============================================================================================
// Process GPS data

static uint32_t GPS_Idle=0;

static int GPS_Copy(void)
{ int Count=0;
  for( ; ; )
  { if(GPS.available()<=0) break;
    uint8_t Byte=GPS.read();
    GPS.encode(Byte); Serial.write(Byte);
    Count++; }
  return Count; }

static void GPS_Display(void)
{ Display.clear();
  // Display.setFont(ArialMT_Plain_10);
  Display.setFont(ArialMT_Plain_16);
  char Line[16];
  if(GPS.time.isValid())
  { int Idx = sprintf(Line, "%02d:%02d:%02d", GPS.time.hour(), GPS.time.minute(), GPS.time.second() /* , GPS.time.centisecond() */ );
    Line[Idx] = 0; }
  else { strcpy(Line, "--:--:--.--"); }
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.drawString(128, 0, Line);
  if(GPS.date.isValid())
  { int Idx = sprintf(Line, "%02d.%02d.%02d", GPS.date.day(), GPS.date.month(), GPS.date.year()-2000);
    Line[Idx] = 0; }
  else { strcpy(Line, "--.--.----"); }
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 0, Line);
  // if(GPS.pos.isValid())
  // { }
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

static uint16_t InfoParmIdx = 0;            // the round-robin index to info records in info packets

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

static int getInfoPacket(OGN1_Packet &Packet)
{ Packet.HeaderWord = 0;
  Packet.Header.Address = Parameters.Address;
  Packet.Header.AddrType = Parameters.AddrType;
  Packet.Header.NonPos = 1;
  Packet.calcAddrParity();
  return ReadInfo(Packet); }

// ===============================================================================================
// Radio

// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
static const uint8_t OGN1_SYNC[10] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A, 0x00, 0x00 };
// OGNv2 SYNC:       0xF56D3738 encoded in Machester
static const uint8_t OGN2_SYNC[10] = { 0x55, 0x99, 0x96, 0x59, 0xA5, 0x95, 0xA5, 0x6A, 0x00, 0x00 };

static const uint8_t PAW_SYNC [10] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x18, 0x71, 0x00, 0x00 };

static RadioEvents_t Radio_Events;

static void Radio_TxDone(void)
{ Serial.printf("%d: Radio_TxDone()\n", millis());
  Radio.Rx(0); }

static void Radio_TxTimeout(void)
{ Serial.printf("%d: Radio_TxTimeout()\n", millis());
  Radio.Rx(0); }

static void Radio_RxDone( uint8_t *Packet, uint16_t Size, int16_t RSSI, int8_t SNR)
{ Serial.printf("%d: Radio_RxDone( , %d, %d, %d)\n", millis(), Size, RSSI, SNR);
}

static void OGN_TxConfig(void)
{ Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower, 50000, 0, 100000, 0, 1, 1, 0, 0, 0, 0, 20, 8, OGN1_SYNC); }

static void PAW_TxConfig(void)
{ Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower+6, 9600, 0, 38400, 0, 1, 1, 0, 0, 0, 0, 20, 8, PAW_SYNC); }

static void OGN_RxConfig(void)
{ Radio.SetRxConfig(MODEM_FSK, 250000, 100000, 0, 250000, 1, 100, 1, 52, 0, 0, 0, 0, true, 8, OGN1_SYNC); }

static int Radio_Transmit(const uint8_t *Data, uint8_t Len=26)
{ uint8_t Packet[2*Len];
  uint8_t PktIdx=0;
  for(uint8_t Idx=0; Idx<Len; Idx++)
  { uint8_t Byte=Data[Idx];
    Packet[PktIdx++]=ManchesterEncode[Byte>>4];                   // software manchester encode every byte
    Packet[PktIdx++]=ManchesterEncode[Byte&0x0F];
  }
  Radio.Send(Packet, 2*Len);
  return 0; }

static int Radio_Transmit(OGN_TxPacket<OGN1_Packet> &TxPacket)
{ return Radio_Transmit(TxPacket.Byte(), TxPacket.Bytes); }

// ===============================================================================================

void setup()
{ // delay(2000); // prevents USB driver crash on startup, do not omit this

  VextON();                   // Turn on power to OLED
  delay(100);

  Serial.begin(115200);                   // console/debug UART
  // while (!Serial) { }                  // wait for USB serial port to connect
  Serial.println("OGN Tracker on HELTEC CubeCell");

  pinMode(P3_3,INPUT);                    // push button

  Pixels.begin();                         // RGB LED
  Pixels.clear();
  Pixels.setBrightness(0x08);
  LED_Green();
  // Pixels.setPixelColor( 0, 255, 0, 0, 0); // Green
  // Pixels.show();

  Parameters.ReadFromFlash();             // read parameters from Flash

  Display.init();                                   // OLED
  Display.clear();
  Display.display();
  Display.setTextAlignment(TEXT_ALIGN_CENTER);      // 
  Display.setFont(ArialMT_Plain_16);
  Display.drawString(64, 32-16/2, "OGN-Tracker");
  Display.display();

  GPS.begin(38400);                                  // GPS

  Radio_Events.TxDone    = Radio_TxDone;             // TRX
  Radio_Events.TxTimeout = Radio_TxTimeout;
  Radio_Events.RxDone    = Radio_RxDone;
  Radio.Init(&Radio_Events);
  Radio.SetChannel(868400000);
  OGN_TxConfig();
  OGN_RxConfig();

}

static OGN_TxPacket<OGN1_Packet> TxPacket;

static bool GPS_Done=0;

void loop()
{
  CONS_Proc();

  if(GPS_Copy()==0) { GPS_Idle++; delay(1); }
               else { GPS_Idle=0; }
  if(GPS_Done)
  { if(GPS_Idle<3) { GPS_Done=0; LED_Blue(); } }
  else
  { if(GPS_Idle>5)
    { if(GPS.date.isValid()) LED_Green();
                       else  LED_Yellow();
      // here we start the slot just after the GPS sent its data
      // printf("%d: Batt\n", millis());
      uint16_t BattVolt = getBatteryVoltage();                    // [mV]
      // printf("BattVolt=%d mV\n", BattVolt);
      // printf("%d: OLED\n", millis());
      GPS_Display();                                              //
      if(getInfoPacket(TxPacket.Packet))
      { TxPacket.Packet.Whiten(); TxPacket.calcFEC();
        // printf("%d: Radio\n", millis());
        Radio_Transmit(TxPacket); }
      // printf("%d:\n", millis());
      LED_OFF();
      GPS_Done=1; }
  }


}
