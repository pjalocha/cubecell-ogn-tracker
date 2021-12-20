#include "Arduino.h"
#include <Wire.h>  

#include "GPS_Air530.h"
#include "GPS_Air530Z.h"

#include "HT_SSD1306Wire.h"
#include "CubeCell_NeoPixel.h"

// #include "EEPROM.h"

#include "format.h"
#include "nmea.h"

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

static Air530Class GPS;                                                       // GPS
// Air530ZClass GPS;

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
  // save Parameters to Flash
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

void setup()
{ // delay(2000); // prevents USB driver crash on startup, do not omit this

  VextON();                   // Turn on power to OLED
  delay(100);

  Serial.begin(115200);                   // console/debug UART
  // while (!Serial) { }                  // wait for USB serial port to connect
  Serial.println("OGN Tracker on HELTEC CubeCell");

  // pinMode(RGB, OUTPUT);

  Pixels.begin();
  Pixels.clear();
  Pixels.setBrightness(0x08);
  Pixels.setPixelColor( 0, 255, 0, 0, 0);
  Pixels.show();

  Display.init();
  Display.clear();
  Display.display();

  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.setFont(ArialMT_Plain_16);
  Display.drawString(64, 32-16/2, "OGN-Tracker");
  Display.display();

  GPS.begin(9600);           // GPS

  // EEPROM.begin(512);         //

  Parameters.setDefault();

}

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
      GPS_Display();
      GPS_Done=1; }
  }


}
