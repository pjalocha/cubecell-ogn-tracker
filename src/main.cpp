// OGN-Tracker for the CubeCell HELTEC modul with GPS and small OLED.

// #define WITH_ADSL  // now works correctly
// #define WITH_FANET // only up to 8000m altitude

// the following options do not work correctly yet in this code

// use either WITH_BMP280 or WITH_BME280 but not both at the same time
// #define WITH_BME280 // read a BME280 pressure/temperature/humidity sensor attached to the I2C (same as the OLED)
// #define WITH_BMP280 // read a BMP280 pressure/temperature sensor attached to the I2C (same as the OLED)

// #define WITH_GPS_CONS // send the GPS NMEA to the console

#include "Arduino.h"
#undef min                    // those are defined in Arduino and cause conflict with #include <slgorithm>
#undef max

#include <Wire.h>

#include "innerWdt.h"         // Watch-Dog Timer

#include "LoRaWan_APP.h"
#include "sx126x.h"

// #include <EEPROM.h>        // there is no real EEPROM: it is emulated by the flash

// #include "GPS_Air530.h"
#include "GPS_Air530Z.h"
#include "gps-satlist.h"

#include "HT_SSD1306Wire.h"
#include "CubeCell_NeoPixel.h"

#ifdef WITH_BMX280
#include "bme280.h"
#endif

#ifdef WITH_BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>  // pressure/temperature/humidity sensor
#endif

#ifdef WITH_BMP280
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>  // pressure/temperature/humidity sensor
#endif

// #include "fifo.h"
// #include "lowpass2.h"
// #include "atmosphere.h"
#include "format.h"
#include "nmea.h"
#include "manchester.h"
#include "ogn1.h"
#include "crc1021.h"
#include "freqplan.h"
#include "radio.h"

#include "main.h"

#ifdef WITH_ADSL
#include "adsl.h"
#endif
#ifdef WITH_FANET
#include "fanet.h"
#endif
#ifdef WITH_MESHT
#include "mesht-proto.h"
#include "mesht.h"
#endif

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

Random64 Random = { 0x0123456789ABCDEF };

static int RNG(uint8_t *Data, unsigned Size)  // produce random bytes
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

uint64_t getUniqueID(void) { return getID(); }        // get unique serial ID of the CPU/chip
uint32_t getUniqueAddress(void) { return getID()&0x00FFFFFF; }

FlashParameters Parameters;       // parameters stored in Flash: address, aircraft type, etc.

// ===============================================================================================

static void Button_ForceActive(void);

// ===============================================================================================

static uint16_t BattVoltage  = 0;        // [mV] battery voltage, measured every second
static uint8_t  BattCapacity = 0;        // [ %]

static uint8_t calcBattCapacity(uint16_t mVolt)
{ if(mVolt>=4100) return 100;                                 // 4.10V or above => full capacity
  if(mVolt<=3500) return   0;                                 // 3.50V or below => zero capacity
  return (mVolt-3500+3)/6; }                                  // linear interpolation (simplified)

// static uint8_t BattCapacity(void) { return BattCapacity(BattVoltage); }

static void ReadBatteryVoltage(void)
{ uint16_t Volt  = getBatteryVoltage();                         // [mv] measure the battery voltage (average over 50 readouts)
  if(BattVoltage==0) BattVoltage = Volt;
  else BattVoltage = (BattVoltage*3+Volt+2)/4;
  BattCapacity = calcBattCapacity(BattVoltage); }

// ===============================================================================================
// GPS: apparently Air530Z is a very different device from Air530, does not look compatible at all

const int GPS_BaudRate = 57600;
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

static uint8_t I2C_Read(TwoWire &Wire, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait)
{ // Serial.printf("I2C_Read(%d, x%02X, x%02X, , [%d], %dms)\n", Bus, Addr, Reg, Len, Wait);
  Wire.beginTransmission(Addr);
  int Ret=Wire.write(Reg); if(Ret!=1) { Wire.endTransmission(true); return 0xD; }
  Ret=Wire.endTransmission(false); if(Ret) return Ret;   // return non-zero on error
  Ret=Wire.requestFrom(Addr, Len);                       // returns the number of bytes returned from the device
  // if(Ret!=Len) Serial.printf("I2C_Read(, x%02X, x%02X, ...) %d=>%d\n", Addr, Reg, Len, Ret);
  if(Ret==Len) Ret=0;
          else Ret=0xE;
  for(uint8_t Idx=0; Idx<Len; Idx++)
  { int Byte=Wire.read(); if(Byte<0) { Ret=0xF; break; }
    Data[Idx]=Byte; }
  // Serial.printf("I2C_Read() => %d:%d [%d]\n", Ret, Idx, Len);
  return Ret; }

static uint8_t I2C_Write(TwoWire &Wire, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait)
{ Wire.beginTransmission(Addr);
  int Ret=Wire.write(Reg); if(Ret!=1) { Wire.endTransmission(true); return 0xD; }
  uint8_t Idx=0;
  for( ; Idx<Len; Idx++)
  { Ret=Wire.write(Data[Idx]); if(Ret!=1) break; }
  Ret=Wire.endTransmission();
  return Ret; }

uint8_t I2C_Read(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait)
{ if(Bus!=0) return 0xB;
  uint8_t Ret=I2C_Read(Wire, Addr, Reg, Data, Len, Wait);
  // Serial.printf("I2C_Read(%d, x%02X, x%02X, , [%d], ) => %d\n", Bus, Addr, Reg, Len, Ret);
  return Ret; }

uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait)
{ if(Bus!=0) return 0xB;
  uint8_t Ret=I2C_Write(Wire, Addr, Reg, Data, Len, Wait);
  return Ret; }


#ifdef WITH_BMX280
static BME280   Baro;

static void BMX280_Init(void)
{ Baro.Bus=0;
  uint8_t Err=Baro.CheckID();
  if(Err==0) Err=Baro.ReadCalib();
  if(Err==0) Err=Baro.Acquire();
  if(Err==0) Baro.Calculate(); }

static void BMX280_Read(GPS_Position &GPS)       // read the pressure/temperature/humidity and put it into the given GPS record
{ if(Baro.ADDR==0) BMX280_Init();
  uint8_t Err=Baro.Acquire();
  if(Err!=0) { Baro.ADDR=0; return; }
  Baro.Calculate();
  GPS.Temperature = Baro.Temperature;            // [0.1degC]
  GPS.Pressure = Baro.Pressure;                  // [0.25Pa]
  GPS.StdAltitude = floorf(BaroAlt(0.25f*GPS.Pressure)*10+0.5);
  // GPS.StdAltitude = Atmosphere::StdAltitude((GPS.Pressure+2)>>2);;
  GPS.hasBaro=1;
  if(!Baro.hasHumidity()) return;
  GPS.Humidity=Baro.Humidity;
  GPS.hasHum=1; }

#endif

// ===============================================================================================
// BME/BMP280 with Arduino library

#ifdef WITH_BME280                               // this works strictly for BME280, although could possibly be backward compatible to BMP280
static Adafruit_BME280 BME280;                   // pressure/temperature/humidity sensor
static uint8_t BME280_Addr = 0x00;

static void BME280_Init(void)
{ for(uint8_t Addr=0x76; Addr<=0x77; Addr++)
  { if(BME280.begin(0x76, &Wire)) { BME280_Addr=Addr; break; } // BME280 on the I2C
    Serial.printf("BME280 not detected at 0x%02X\n", Addr); }
}

static void BME280_Read(GPS_Position &GPS)       // read the pressure/temperature/humidity and put it into the given GPS record
{ if(BME280_Addr==0) return;
  float Temp  = BME280.readTemperature();        // [degC]
  GPS.Temperature = floorf(Temp*10+0.5);         // [0.1 degC]
  float Press = BME280.readPressure();           // [Pa]
  GPS.Pressure    = floorf(Press*4+0.5);         // [1/4 Pa]
  // GPS.StdAltitude = Atmosphere::StdAltitude((GPS.Pressure+2)>>2);;
  GPS.StdAltitude = floorf(BaroAlt(Press)*10+0.5);
  GPS.hasBaro=1;
  float Hum   = BME280.readHumidity();           // [%]
  GPS.Humidity    = floorf(Hum*10+0.5);          // [0.1 %]
  GPS.hasHum=1; }
#endif

#ifdef WITH_BMP280                               // this works strictly for BMP280
static Adafruit_BMP280 BMP280(&Wire);            // pressure/temperature/humidity sensor
static uint8_t BMP280_Addr = 0x00;

static void BMP280_Init(void)
{ for(uint8_t Addr=0x76; Addr<=0x77; Addr++)
  { if(BMP280.begin(0x76)) { BMP280_Addr=Addr; break; } // BMP280 on the I2C
    Serial.printf("BMP280 not detected at 0x%02X\n", Addr); }
}

static void BMP280_Read(GPS_Position &GPS)       // read the pressure/temperature/humidity and put it into the given GPS record
{ if(BMP280_Addr==0) return;
  float Temp  = BMP280.readTemperature();        // [degC]
  GPS.Temperature = floorf(Temp*10+0.5);          // [0.1 degC]
  float Press = BMP280.readPressure();           // [Pa]
  GPS.Pressure    = floorf(Press*4+0.5);          // [1/4 Pa]
  // GPS.StdAltitude = Atmosphere::StdAltitude((GPS.Pressure+2)>>2);;
  GPS.StdAltitude = floorf(BaroAlt(Press)*10+0.5);
  GPS.hasBaro=1; }
#endif

// ===============================================================================================

static SSD1306Wire Display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10 ); // OLED: addr , freq , i2c group , resolution , rst

static CubeCell_NeoPixel Pixels(1, RGB, NEO_GRB + NEO_KHZ800);                // three-color LED

// ===============================================================================================

static char Line[256];                     // for OLED and console printing

static void VextON(void)                   // Vext controls OLED power
{ pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW); }

static void VextOFF(void)                  // Vext default OFF
{ pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH); }

// ===============================================================================================

const int RelayQueueSize = 16;
__attribute__((aligned(4)))
static Relay_PrioQueue<OGN_RxPacket<OGN1_Packet>, RelayQueueSize> OGN_RelayQueue;  // OGN candidate packets to be relayed
#ifdef WITH_ADSL
static Relay_PrioQueue<ADSL_RxPacket, RelayQueueSize>            ADSL_RelayQueue;  // ADSL candidate packets to be relayed
#endif

static const OGN_RxPacket<OGN1_Packet> *FindTarget(uint32_t Target, uint8_t &TgtIdx)
{ for( uint8_t Idx=TgtIdx; ; )
  { const OGN_RxPacket<OGN1_Packet> *Packet = OGN_RelayQueue.Packet+Idx;
    if(Packet->Alloc && !Packet->Packet.Header.NonPos && Packet->Packet.getAddressAndType()==Target) { TgtIdx=Idx; return Packet; }
    Idx++; if(Idx>=RelayQueueSize) Idx=0;
    if(Idx==TgtIdx) break; }
  return 0; }

static void CleanRelayQueue(uint32_t Time, uint32_t Delay=12) // remove "old" packets from the relay queue
{ uint8_t Sec = (Time-Delay)%60; // Serial.printf("cleanTime(%d)\n", Sec);
  OGN_RelayQueue.cleanTime(Sec);               // remove packets 20(default) seconds into the past
#ifdef WITH_ADSL
  uint8_t qSec = Sec%15;
  ADSL_RelayQueue.cleanTime(qSec<<2);
#endif
}

static bool GetRelayPacket(OGN_TxPacket<OGN_Packet> *Packet)      // prepare a packet to be relayed
{ if(OGN_RelayQueue.Sum==0) return 0;                     // if no packets in the relay queue
  XorShift64(Random.Word);                             // produce a new random number
  uint8_t Idx=OGN_RelayQueue.getRand(Random.RX);          // get weight-random packet from the relay queue
  if(OGN_RelayQueue.Packet[Idx].Rank==0) return 0;        // should not happen ...
  memcpy(Packet->Packet.Byte(), OGN_RelayQueue[Idx]->Byte(), OGN_Packet::Bytes); // copy the packet
  Packet->Packet.Header.Relay=1;                      // increment the relay count (in fact we only do single relay)
  // Packet->Packet.calcAddrParity();
  if(!Packet->Packet.Header.Encrypted) Packet->Packet.Whiten(); // whiten but only for non-encrypted packets
  Packet->calcFEC();                                  // Calc. the FEC code => packet ready for transmission
  // PrintRelayQueue(Idx);  // for debug
  OGN_RelayQueue.decrRank(Idx);                           // reduce the rank of the packet selected for relay
  return 1; }

// ===============================================================================================
// CONSole UART

static void CONS_UART_Write(char Byte) // write byte to the console (USB serial port)
{ Serial.write(Byte); }

static int  CONS_UART_Free(void)
{ return Serial.availableForWrite(); }

static int  CONS_UART_Read (uint8_t &Byte)
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

static void ConsNMEA_Process(void)                              // process NMEA received on the console
{ if(!ConsNMEA.isPOGNS()) return;                               // ignore all but $POGNS
  if(ConsNMEA.hasCheck() && !ConsNMEA.isChecked() ) return;     // if CRC present then it must be correct
  if(ConsNMEA.Parms==0) { PrintPOGNS(); return; }               // if no parameters given, print the current parameters as $POGNS
  // printf("ConsNMEA_Process() - before .ReadPOGNS()\n\r" );
  Parameters.ReadPOGNS(ConsNMEA);                               // read parameter values given in $POGNS
  // printf("ConsNMEA_Process() - after .ReadPOGNS()\n\r" );
  // PrintParameters();                                            // print the new parameter values
  Parameters.WriteToFlash(); }                                  // write new parameter set to flash

// static void CONS_CtrlB(void) { Serial.printf("Battery: %5.3fV %d%%\n", 0.001*BattVoltage, BattCapacity); }

static void CONS_CtrlC(void)
{ PrintParameters(); }

static void CONS_CtrlR(void)
{ uint8_t Len=Format_String(Line, "Relay: ");
  Len+=OGN_RelayQueue.Print(Line+Len);
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
  if(Count) Button_ForceActive();                                   // if characters received on the console then activate the OLED
  return Count; }

// ===============================================================================================
// Process GPS satelite data

static GPS_SatList GPS_SatMon;                   // list of satellites for SNR monitoring

static uint8_t    GPS_SatSNR = 0;                // [0.25dB] average SNR from the GSV sentences
static uint8_t    GPS_SatCnt = 0;                // number of satelites in the above average

// ===============================================================================================
// Process GPS position data

static int          GPS_Ptr = 0;       // 
static GPS_Position GPS_Pipe[2];       // two most recent GPS readouts

static uint32_t GPS_ValidUTC = 0;      // [sec]        when the last position of the GPS was recorded
static  int32_t GPS_Latitude = 0;      // [1/60000deg]
static  int32_t GPS_Longitude = 0;     // [1/60000deg]
static  int32_t GPS_Altitude = 0;      // [0.1m]
static  int16_t GPS_GeoidSepar= 0;     // [0.1m]
static uint16_t GPS_LatCosine = 3000;  // [1.0/4096]
static uint8_t  GPS_Satellites = 0;    //

uint32_t GPS_PPS_ms = 0;                        // [ms] System timer at the most recent PPS
uint32_t GPS_PPS_UTC = 0;                      // [sec] Unix time which corresponds to the most recent PPS

static uint32_t GPS_Idle=0;                            // [ticks] to detect when GPS stops sending data

static union
{ uint32_t Flags;
  struct
  { bool PowerON   :1;                                 // is turned ON
    bool BurstDone :1;                                 // data burst is complete
    bool TimeValid :1;                                 // time is valid
    bool DateValid :1;                                 // date is valid
    bool  FixValid :1;                                 // time/position fix is valid
    bool SlotDone  :1;
  } ;
} GPS_State = { 0 };                                   // single bit state flags

static NMEA_RxMsg GpsNMEA;                             // NMEA catcher for GPS

static int GPS_Process(void)                           // process serial data stream from the GPS
{ int Count=0;
  for( ; ; )
  { if(GPS.available()<=0) break;                      // if no more characters then give up
    uint8_t Byte=GPS.read();                           // read character
    GpsNMEA.ProcessByte(Byte);                         // NMEA interpreter
    if(GpsNMEA.isComplete())                           // if NMEA is done
    { GPS_SatMon.Process(GpsNMEA);                     // process satellite data
      GPS_Pipe[GPS_Ptr].ReadNMEA(GpsNMEA);             // interpret the position NMEA by the GPS
#ifndef WITH_GPS_NMEA_PASS
      if(GpsNMEA.isGxRMC() || GpsNMEA.isGxGGA() /* || GpsNMEA.isGxGSA() */ )          // selected GPS NMEA sentences
#endif
      { GpsNMEA.Data[GpsNMEA.Len]=0; Serial.println((const char *)(GpsNMEA.Data)); }  // copy to the console
      GpsNMEA.Clear(); }
// #ifdef WITH_GPS_CONS                                   // char-by-char copy of the GPS output to the console
//     Serial.write(Byte);                                // copy character to the console (no loss of characters here)
// #endif
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

static bool GPS_ReadPPS(void) { return digitalRead(GPIO12); } // tell if PPS line of the GPS is HIGH

static void GPS_HardPPS(void)       // called by hardware interrupt on PPS
{ // uint32_t msTime = millis()-GPS_PPS_ms;
  // Serial.printf("PPS: %4d\n", msTime);
}

// ===============================================================================================
// OLED pages

static uint8_t OLED_isON=0;

static void OLED_ON(void)
{ if(OLED_isON) return;
  Display.wakeup(); OLED_isON=1; }

static void OLED_OFF(void)
{ if(!OLED_isON) return;
  Display.sleep(); OLED_isON=0; }

static void OLED_ConfirmPowerON(void)                                     // 
{ Display.clear();
  Display.setFont(ArialMT_Plain_16);
  Display.setTextAlignment(TEXT_ALIGN_CENTER);
  Display.drawString(64, 12, "Confirm");
  Display.drawString(64, 36, "Power-ON");
  Display.display(); }

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
  Display.drawString( 0,  0, "Mini-OGN");

  Parameters.Print(Line); Line[10]=0;
  Display.drawString( 0, 16, Line+2);

  Display.setFont(ArialMT_Plain_10);
#ifdef SOFT_NAME
  Display.drawString( 0, 44, SOFT_NAME);
#endif
  Display.drawString( 0, 54, "(c) Pawel Jalocha");
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

static const char *AcftTypeName[16] = { "----", "Glid", "Tow ", "Heli",
                                        "SkyD", "Drop", "Hang", "Para",
                                        "Pwrd", "Jet ", "UFO",  "Ball",
                                        "Zepp", "UAV",  "Car ", "Fix " } ;

static const char *CardDir[16] = { "N  ", "NNE", "NE ", "ENE",
                                   "E  ", "ESE", "SE ", "SSE",
                                   "S  ", "SSW", "SW ", "WSW",
                                   "W  ", "WNW", "NW ", "NW " } ;

static uint8_t Format_CardDir(char *Out, uint8_t Dir)
{ Dir+=0x10; Dir>>=4; memcpy(Out, CardDir[Dir], 3); return 3; }

static void OLED_Target(uint32_t Target, uint8_t &TgtIdx, const GPS_Position &GPS)
{ Display.clear();
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  if(Target)
  { uint8_t Len=Format_String(Line, "----:");
    Line[Len++]=HexDigit(Target>>24);
    Line[Len++]=':';
    Len+=Format_Hex(Line+Len, (uint8_t)(Target>>16));
    Len+=Format_Hex(Line+Len, (uint16_t)Target);
    Line[Len]=0; // Display.drawString(0, 0, Line);
    const OGN_RxPacket<OGN1_Packet> *Packet = FindTarget(Target, TgtIdx);
    if(Packet)
    { uint8_t Len=Format_String(Line, AcftTypeName[Packet->Packet.Position.AcftType]);
      Line[Len++]=':';
      Line[Len++]=HexDigit(Packet->Packet.Header.AddrType);
      Line[Len++]=':';
      Len+=Format_Hex(Line+Len, (uint8_t)(Packet->Packet.Header.Address>>16));
      Len+=Format_Hex(Line+Len, (uint16_t)Packet->Packet.Header.Address);
      Line[Len++]=' ';
      Line[Len++]=' ';
      Len+=Format_SignDec(Line+Len, (int32_t)Packet->RxRSSI*(-5), 2, 1);
      Len+=Format_String(Line+Len, "dBm ");
      Line[Len]=0; Display.drawString(0, 0, Line);
      Display.setFont(ArialMT_Plain_16);
      uint32_t Dist= IntDistance(Packet->LatDist, Packet->LonDist);      // [m]
      uint32_t Dir = IntAtan2(Packet->LonDist, Packet->LatDist);         // [16-bit cyclic]
      Dir &= 0xFFFF; Dir = (Dir*360)>>16;                                // [deg]
      Len=Format_UnsDec(Line, Dir, 3);                                   // [deg] direction to target
      // Len=Format_CardDir(Line, Dir>>8);
      // Line[Len++]=0xb0;
      Line[Len++]=' ';
      Len+=Format_UnsDec(Line+Len, (Dist+50)/100, 2, 1);                 // [km] distance to target
      Len+=Format_String(Line+Len, "km ");
      int32_t Alt = Packet->Packet.DecodeAltitude();
      Len+=Format_UnsDec(Line+Len, (uint32_t)Alt);
      Line[Len++]='m';
      Line[Len]=0; Display.drawString(0, 20, Line);
      int32_t AltDiff = Packet->Packet.DecodeAltitude()*10-GPS.Altitude;
      int32_t Climb = Packet->Packet.DecodeClimbRate()*GPS.ClimbRate;
      if(Packet->Packet.hasBaro() && GPS.hasBaro)
      { AltDiff = Packet->Packet.DecodeStdAltitude()*10 - GPS.StdAltitude; }
      Len=Format_SignDec(Line, AltDiff, 2, 1);
      Line[Len++]='m'; Line[Len++]=' ';
      Len+=Format_SignDec(Line+Len, Climb, 2, 1);
      Len+=Format_String(Line+Len, "m/s ");
      Line[Len]=0; Display.drawString(0, 35, Line);
      Display.setFont(ArialMT_Plain_10);
      uint16_t Speed = Packet->Packet.DecodeSpeed();    // [0.1m/s]
      uint16_t Track = Packet->Packet.DecodeHeading();  // [0.1deg]
      Len=Format_UnsDec(Line, (uint32_t)Track/10, 3);      // [deg] direction to target
      // Line[Len++]=0xb0;
      // Len=Format_CardDir(Line, Packet->Packet.Position.Heading>>2);
      Line[Len++]=' ';
      Len+=Format_UnsDec(Line+Len, (uint32_t)Speed, 2, 1);                 // [km] distance to target
      Len+=Format_String(Line+Len, "m/s  ");
      int8_t Age = Packet->Packet.Position.Time;
      if(Age<60)
      { Age = Age-GPS.Sec;
        if(Age>3) Age-=60;
        Len+=Format_SignDec(Line+Len, (int32_t)Age);
        Line[Len++]='s'; }
      Line[Len]=0; Display.drawString(0, 50, Line);

      // Line[Len]=0; Display.drawString(0, 50, Line);
    }
    else Display.drawString(0, 0, Line);
  }
  else
  { Display.setFont(ArialMT_Plain_16);
    Display.drawString(0, 0, "No targets in range"); }

  Display.display(); }

static void OLED_Relay(const GPS_Position &GPS)                 // display list of aircrafts on the Relay list
{
  Display.clear();
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  uint8_t VertPos=0;
  for( uint8_t Idx=0; Idx<RelayQueueSize; Idx++)
  { OGN_RxPacket<OGN1_Packet> *Packet = OGN_RelayQueue.Packet+Idx; if(Packet->Alloc==0) continue;
    if(Packet->Packet.Header.NonPos) continue;                     // don't show non-position packets
    uint32_t Dist= IntDistance(Packet->LatDist, Packet->LonDist);      // [m]
    uint32_t Dir = IntAtan2(Packet->LonDist, Packet->LatDist);         // [16-bit cyclic]
    Dir &= 0xFFFF; Dir = (Dir*360)>>16;                                // [deg]
    uint8_t Len=0;
    Len+=Format_String(Line+Len, AcftTypeName[Packet->Packet.Position.AcftType]);
    Line[Len++]=' ';
    Len+=Format_UnsDec(Line+Len, (uint32_t)Packet->Packet.DecodeAltitude()); // [m] altitude
    Line[Len++]='m'; Line[Len++]=' ';
    Len+=Format_UnsDec(Line+Len, Dir, 3);                             // [deg] direction to target
    // Len+=Format_CardDir(Line+Len, Dir>>8);
    Line[Len++]=' ';
    Len+=Format_UnsDec(Line+Len, (Dist+50)/100, 2, 1);                // [km] distance to target
    Len+=Format_String(Line+Len, "km");
    Line[Len]=0;
    Display.drawString(0, VertPos, Line);
    VertPos+=10; if(VertPos>=64) break; }
  if(VertPos==0)
  { Display.setFont(ArialMT_Plain_16);
    Display.setTextAlignment(TEXT_ALIGN_LEFT);
    Display.drawString(0, 0, "No relay cand."); }
  Display.display(); }

static void OLED_GPS(const GPS_Position &GPS)                 // display time, date and GPS data/status
{ Display.clear();
  Display.setFont(ArialMT_Plain_16);
  if(GPS.isTimeValid())
  { uint8_t Len = GPS.FormatDate_DDMMYY(Line, '.'); }
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
    { if(GPS.isValid()) Len+=Format_UnsDec(Line+Len, (uint32_t)GPS.Satellites);
                  else  Len+=Format_UnsDec(Line+Len, (uint32_t)GPS_SatCnt);
      Line[Len++]='s'; Line[Len++]='a'; Line[Len++]='t'; }
    else
    { Len+=Format_UnsDec(Line+Len, ((uint32_t)GPS_SatSNR+2)/4, 2);
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
    if(GPS.Sec&1) { Len+=Format_UnsDec(Line+Len, ((uint32_t)BattVoltage+5)/10, 3, 2); Line[Len++]= 'V'; }
    else
    { Len+=Format_UnsDec(Line+Len, (uint32_t)BattCapacity); Line[Len++]= '%'; }
    Line[Len]=0;
    Display.setTextAlignment(TEXT_ALIGN_RIGHT);
    Display.drawString(128, 48, Line); }
  Display.display(); }

static void OLED_Baro(const GPS_Position &GPS)                 // display pressure sensor info and data
{ Display.clear();
  Display.setFont(ArialMT_Plain_16);

  uint8_t Len=Format_String(Line, "No baro sensor");
  bool hasBaro=0;
#ifdef WITH_BMX280
   if(Baro.ADDR) { Len=sprintf(Line, "BM%280 ", Baro.hasHumidity()?'E':'P'); hasBaro=1; }
#endif
#ifdef WITH_BMP280
  if(BMP280_Addr) { Len=Format_String(Line, "BMP280 "); hasBaro=1; }
#endif
#ifdef WITH_BME280
  if(BME280_Addr) { Len=Format_String(Line, "BME280 "); hasBaro=1; }
#endif
  if(hasBaro)
  { if(GPS.hasBaro)
    { Len+=Format_UnsDec(Line+Len, GPS.Pressure/4, 5, 2);
      Len+=Format_String(Line+Len, "hPa "); }
    else Len+=Format_String(Line+Len, "----.--hPa ");
  }
  Line[Len]=0;
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 0, Line);
#if defined(WITH_BMP280) || defined(WITH_BME280) || defined(WITH_BMX280)
  if(hasBaro)
  { Len=0;
    if(GPS.hasBaro)
    { Len+=Format_SignDec(Line+Len, (int32_t)GPS.StdAltitude, 5, 1);
      Len+=Format_String(Line+Len, "m ");
      Len+=Format_SignDec(Line+Len, (int32_t)GPS.ClimbRate, 2, 1);
      Len+=Format_String(Line+Len, "m/s "); }
    else Len+=Format_String(Line+Len, "-----.-m  --.-m/s ");
    Line[Len]=0;
    Display.drawString(0, 16, Line);
    Len=0;
    if(GPS.hasBaro)
    { Len+=Format_SignDec(Line+Len, (int32_t)GPS.Temperature, 2, 1);
      Line[Len++]=0xB0;
      Line[Len++]='C';
      Line[Len++]=' ';
      if(GPS.hasHum)
      { Len+=Format_SignDec(Line+Len, (int32_t)GPS.Humidity, 2, 1, 1);
        Line[Len++]='%'; Line[Len++]=' '; }
    }
    else Len+=Format_String(Line+Len, "---.- C --.-% ");
    Line[Len]=0;
    Display.drawString(0, 32, Line); }
#endif
  Display.display(); }

static void OLED_RF(void)                 // display RF-related data
{ Display.clear();
  Display.setFont(ArialMT_Plain_16);

  uint8_t Len=Format_String(Line, "SX1262: ");
  Len+=Format_SignDec(Line+Len, (int32_t)Parameters.TxPower);              // Tx power
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

  Len=0; uint8_t Acfts = OGN_RelayQueue.size();
  if(Acfts)
  { Len+=Format_UnsDec(Line+Len, (uint32_t)Acfts);
    Len+=Format_String(Line+Len, " Acft");
    Line[Len]=0;
    Display.setTextAlignment(TEXT_ALIGN_LEFT);
    Display.drawString(0, 32, Line); }

  Len=0;
  Len+=Format_UnsDec(Line+Len, (uint32_t)RX_OGN_Count64);
  Len+=Format_String(Line+Len, "/min");
  Line[Len]=0;
  Display.setTextAlignment(TEXT_ALIGN_RIGHT);
  Display.drawString(128, 32, Line);

  Len=0;
  Len+=Format_String(Line+Len, Radio_FreqPlan.getPlanName());                 // name of the frequency plan
  Line[Len++]=' ';
  Len+=Format_UnsDec(Line+Len, (uint32_t)(Radio_FreqPlan.getCenterFreq()/100000), 3, 1); // center frequency
  Len+=Format_String(Line+Len, "MHz");
  Line[Len]=0;
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.drawString(0, 48, Line);

  Display.display(); }

const  int OLED_Pages    = 5;
static int OLED_CurrPage = 0;

#ifdef WITH_TRACKING
static uint8_t  OLED_Mode=0;            // 0=pages, 1=tracking a target
#else
static const uint8_t  OLED_Mode=0;
#endif

static uint8_t  OLED_CurrTgtIdx = 0;    // index of the current target in the relay queue
static uint32_t OLED_CurrTarget = 0;    // target aircraft being tracked

static void OLED_NextTarget(void)       // switch to the next target
{ for(uint8_t Idx=OLED_CurrTgtIdx; ; )
  { Idx++; if(Idx>=RelayQueueSize) Idx=0; if(Idx==OLED_CurrTgtIdx) break;
    OGN_RxPacket<OGN1_Packet> *Packet = OGN_RelayQueue.Packet+Idx; if(Packet->Alloc==0) continue;
    if(Packet->Packet.Header.NonPos) continue;
    uint32_t AddressAndType = Packet->Packet.getAddressAndType();
    if(AddressAndType!=OLED_CurrTarget)
    { OLED_CurrTarget=AddressAndType; OLED_CurrTgtIdx=Idx; break; }
  }
}

static void OLED_NextPage(void)
{ OLED_CurrPage++;
  if(OLED_CurrPage>=OLED_Pages) OLED_CurrPage=0; }

static void OLED_DispPage(const GPS_Position &GPS)
{ if(!OLED_isON) return;
  if(OLED_Mode)
  { if(OLED_CurrTarget==0) OLED_NextTarget();
    OLED_Target(OLED_CurrTarget, OLED_CurrTgtIdx, GPS);
    return; }
  switch(OLED_CurrPage)
  { case 2: OLED_Info(); break;
    case 1: OLED_RF(); break;
    case 3: OLED_Baro(GPS); break;
    case 4: OLED_Relay(GPS); break;
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
// ADS-L packets

#ifdef WITH_ADSL
static int getPosPacket(ADSL_Packet &Packet, const GPS_Position &GPS)  // encode position into an ADS-L packet
{ Packet.Init();
  Packet.setAddress    (Parameters.Address);
  Packet.setAddrTypeOGN(Parameters.AddrType);
  Packet.setAcftTypeOGN(Parameters.AcftType);
  GPS.Encode(Packet);
  return 1; }

static bool getTelemSatSNR(ADSL_Packet &Packet)
{ Packet.Init(0x42);
  Packet.setAddress    (Parameters.Address);
  Packet.setAddrTypeOGN(Parameters.AddrType);
  Packet.setRelay(0);
  Packet.Telemetry.Header.TelemType=0x3;                            // 3 = GPS telemetry
  Packet.SatSNR.Header.GNSStype=0;                                  // 0 = GPS satellite SNR
  for(uint8_t Sys=0; Sys<5; Sys++)
  { Packet.SatSNR.Data.SatSNR[Sys]=GPS_SatMon.getSysStatus(Sys); }
  Packet.SatSNR.Data.Inbalance = 0;
  Packet.SatSNR.Data.PDOP = GPS_SatMon.PDOP;
  Packet.SatSNR.Data.HDOP = GPS_SatMon.HDOP;
  Packet.SatSNR.Data.VDOP = GPS_SatMon.VDOP;
  return 1; }

static bool GetRelayPacket(ADSL_Packet *Packet)           // prepare a packet to be relayed
{ if(ADSL_RelayQueue.Sum==0) return 0;                    // if no packets in the relay queue
  XorShift32(Random.RX);                                  // produce a new random number
  uint8_t Idx=ADSL_RelayQueue.getRand(Random.RX);         // get weight-random packet from the relay queue
  if(ADSL_RelayQueue.Packet[Idx].Rank==0) return 0;       // should not happen ...
  *Packet = ADSL_RelayQueue[Idx]->Packet;
  Packet->setRelay();
  // Packet->Scramble();
  // Packet->setCRC();
  ADSL_RelayQueue.decrRank(Idx);                           // reduce the rank of the packet selected for relay
  return 1; }

static bool getAdslPacket(ADSL_Packet &Packet, const GPS_Position &GPS)  // produce an ADS-L packet
{ static uint8_t BackOff=0;
  if(BackOff) { BackOff--; return getPosPacket(Packet, GPS); }
  BackOff=29+Random.RX%5;
  return getTelemSatSNR(Packet); }

#endif

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
  Packet.Header.NonPos = 1;                                            // this is not a position packet
  Packet.calcAddrParity();
  Packet.Status.Hardware=HARDWARE_ID;
  Packet.Status.Firmware=SOFTWARE_ID;
  Packet.clrTemperature();                                             // we have no temperature sensor other than the pressure sensor
  GPS.EncodeStatus(Packet);                                            // encode altitude and possibly pressure/temperature
  uint8_t SatSNR = (GPS_SatSNR+2)/4;                                   // encode number of satellites and SNR in the Status packet
  if(SatSNR>8) { SatSNR-=8; if(SatSNR>31) SatSNR=31; }
          else { SatSNR=0; }
  Packet.Status.SatSNR = SatSNR;
  Packet.EncodeVoltage(((BattVoltage<<3)+62)/125);                     // [1/64V]
  Packet.Status.RadioNoise = -RX_RSSI.getOutput();                     // [-0.5dBm]
  uint16_t RxRate = RX_OGN_Count64+1;
  uint8_t RxRateLog2=0; RxRate>>=1; while(RxRate) { RxRate>>=1; RxRateLog2++; }
  Packet.Status.RxRate = RxRateLog2;
  uint8_t TxPower = Parameters.TxPower-4;
  if(TxPower>15) TxPower=15;
  Packet.Status.TxPower = TxPower;
  return 1; }

#ifdef WITH_MESHT

__attribute__((aligned(4)))
static MeshtProto_NodeInfo Mesht_NodeInfo;
__attribute__((aligned(4)))
static MeshtProto_GPS Mesht_GPS;
__attribute__((aligned(4)))
static MeshtProto_GPS Mesht_RefGPS;
__attribute__((aligned(4)))
static AES128 AES;

static uint32_t MeshtHash(uint32_t X)
{ X ^= X>>16;
  X *= 0x85ebca6b;
  X ^= X>>13;
  X *= 0xc2b2ae35;
  X ^= X>>16;
  return X; }

static int getMeshNodeInfo(void)
{ Mesht_NodeInfo.Clear();
  Mesht_NodeInfo.MAC=getUniqueID();
  sprintf(Mesht_NodeInfo.ID,    "!%08x",   (uint32_t)Mesht_NodeInfo.MAC);
  sprintf(Mesht_NodeInfo.Short, "%04x",    (uint16_t)Mesht_NodeInfo.MAC);
  Mesht_NodeInfo.Role=5;                 // 5:tracker
  /// Mesht_NodeInfo.Hardware=??
  sprintf(Mesht_NodeInfo.Name, "OGN:%X:%s:%s", Parameters.AcftType, Parameters.Reg, Parameters.Pilot);
  return 1; }

static int getMeshtGPS(const GPS_Position *Position)
{ Mesht_GPS.Clear();
  Mesht_GPS.Time = Position->getUnixTime();
  if(!Position->isValid()) return 0;
  Mesht_GPS.Lat = (int64_t)Position->Latitude*50/3;
  Mesht_GPS.Lon = (int64_t)Position->Longitude*50/3;
  Mesht_GPS.AltMSL = (Position->Altitude+5)/10; Mesht_GPS.hasAltMSL=Mesht_GPS.AltMSL>0;
  Mesht_GPS.Speed  = (Position->Speed+5)/10; Mesht_GPS.hasSpeed=1;
  Mesht_GPS.Track  =  Position->Heading*10; Mesht_GPS.hasTrack=1;
  Mesht_GPS.Prec_bits=32;
  return 1; }

static int getMeshtPacket(MESHT_Packet *Packet, const GPS_Position *Position)
{ int OK=Packet->setHeader(Parameters.Address, Parameters.AddrType, Parameters.AcftType, getUniqueID(), 5);
  if(!OK) return 0;
  static uint8_t InfoBackOff=0;
  int Len=0;
  OK=getMeshtGPS(Position);                                              // get the GPS position or at least the time
  if(OK) Len=MeshtProto::EncodeGPS(Packet->getMeshtMsg(), Mesht_GPS);
  bool Pos=OK;
  if(InfoBackOff) InfoBackOff--;
  if(!OK || InfoBackOff==0)                                              // decide to send NodeInfo instead of position
  { OK=getMeshNodeInfo();
    if(OK) Len=MeshtProto::EncodeNodeInfo(Packet->getMeshtMsg(), Mesht_NodeInfo);
    InfoBackOff = 7+Random.RX%5;
    Pos=0; }
  if(!OK || Len==0) return 0;
  if(Pos)
  { bool Send=Mesht_GPS.TimeDistLimit(Mesht_RefGPS);
    if(!Send) return 0;
    Mesht_RefGPS=Mesht_GPS; }
  Packet->Len=Packet->HeaderSize+Len;
  Packet->Header.PktID ^= MeshtHash(Packet->Header.Src+Mesht_GPS.Time);  // scramble packet-ID by the hash of MAC and Time
  OK=Packet->encryptMeshtMsg(AES);
  return OK; }
#endif

#ifdef WITH_FANET
static int getFNTpacket(FANET_Packet &Packet, const GPS_Position &GPS) // encode position into a FANET packet
{ if(GPS.Altitude>80000) return 0;                      // FANET altitude limit
  Packet.setAddress(Parameters.Address);
  GPS.EncodeAirPos(Packet, Parameters.AcftType, !Parameters.Stealth);
  return 1; }
#endif

static int getPosPacket(OGN1_Packet &Packet, const GPS_Position &GPS)  // encode position into an OGN packet
{ Packet.HeaderWord = 0;
  Packet.Header.Address = Parameters.Address;                          // aircraft address
  Packet.Header.AddrType = Parameters.AddrType;                        // aircraft address-type
  Packet.calcAddrParity();
  Packet.Position.AcftType = Parameters.AcftType;                      // aircraft type
  Packet.Position.Stealth  = Parameters.Stealth;
  GPS.Encode(Packet);
  return 1; }

// ===============================================================================================

#ifdef WITH_ADSL
static void Radio_RxProcADSL(FSK_RxPacket *RxPkt)
{ uint8_t RxPacketIdx  = ADSL_RelayQueue.getNew();                   // get place for this new packet
  ADSL_RxPacket *RxPacket = ADSL_RelayQueue[RxPacketIdx];
  int CorrErr=RxPkt->ErrCount();
  if(RxPkt->Manchester) CorrErr=ADSL_Packet::Correct(RxPkt->Data, RxPkt->Err);
  if(CorrErr<0) return;
  memcpy(&(RxPacket->Packet.Version), RxPkt->Data, RxPacket->Packet.TxBytes-3);
  RxPacket->RxErr   = CorrErr;
  RxPacket->RxChan  = RxPkt->Channel;
  RxPacket->RxRSSI  = RxPkt->RSSI;
  RxPacket->Correct = 1;
  RxPacket->Packet.Descramble();
  if(!RxPacket->Packet.isPos()) return;
  uint8_t AddrType = RxPacket->Packet.getAddrTable();
  if(AddrType<4) AddrType=0;
  else AddrType-=4;
  uint8_t MyOwnPacket = ( RxPacket->Packet.getAddress()  == Parameters.Address )
                     && (                       AddrType == Parameters.AddrType);
  if(MyOwnPacket) return;                                                             // don't process my own (relayed) packets
  if(GPS_Satellites)                                                                  // if GPS has a fix
  { int32_t LatDist=0, LonDist=0;
    bool DistOK = RxPacket->calcDistanceVector(LatDist, LonDist, GPS_Latitude, GPS_Longitude, GPS_LatCosine)>=0;
    if(DistOK)                                                                          // reasonable reception distance
    { RxPacket->LatDist=LatDist;
      RxPacket->LonDist=LonDist;
      RxPacket->calcRelayRank(GPS_Altitude/10);                                         // calculate the relay-rank (priority for relay)
      ADSL_RxPacket *PrevRxPacket = ADSL_RelayQueue.addNew(RxPacketIdx);                // add to the relay queue and get the previ>
    }
  }
}
#endif

static void Radio_RxProcOGN(FSK_RxPacket *RxPkt)
{ uint8_t RxPacketIdx  = OGN_RelayQueue.getNew();                          // get place for this new packet
  OGN_RxPacket<OGN1_Packet> *RxPacket = OGN_RelayQueue[RxPacketIdx];
  uint8_t DecErr = RxPkt->Decode(*RxPacket, Decoder);                  // LDPC FEC decoder
  // Serial.printf("%d: Radio_RxDone( , %d, %d, %d) RxErr=%d/%d %08X { %08X %08X }\n",
  //         millis(), Size, RSSI, SNR, DecErr, RxPacket->RxErr, RxPacket->Packet.HeaderWord, Random.GPS, Random.RX);
  if(DecErr || RxPacket->RxErr>=10 ) { /* RxFIFO.Read(); LED_OFF(); */ return; }              // if FEC not correctly decoded or too many bit errors then give up
  uint8_t OwnPacket = ( RxPacket->Packet.Header.Address  == Parameters.Address  )       // is it my own packet (through a relay) ?
                   && ( RxPacket->Packet.Header.AddrType == Parameters.AddrType );
  if(OwnPacket || RxPacket->Packet.Header.NonPos || RxPacket->Packet.Header.Encrypted) { RxFIFO.Read(); LED_OFF(); return; }
  RxPacket->Packet.Dewhiten();
  if(GPS_Satellites)                                                  // if GPS has a fix
  { int32_t LatDist=0, LonDist=0;
    bool DistOK = RxPacket->Packet.calcDistanceVector(LatDist, LonDist, GPS_Latitude, GPS_Longitude, GPS_LatCosine)>=0;
    if(DistOK)
    { RxPacket->LatDist = LatDist;
      RxPacket->LonDist = LonDist;
      // int32_t Alt = GPS_Altitude/10; if(Alt<0) Alt=0;
      RxPacket->calcRelayRank(GPS_Altitude/10);                    // calculate the relay-rank (priority for relay)
      // Serial.printf("GPS:%dm Pkt:%dm/%d => Rank:%d\r\n", GPS_Altitude/10, RxPacket->Packet.DecodeAltitude(), RxPacket->RxRSSI, RxPacket->Rank);
      OGN_RxPacket<OGN1_Packet> *PrevRxPacket = OGN_RelayQueue.addNew(RxPacketIdx);
      // uint8_t Len=RxPacket->WritePOGNT(Line);
    }
  }
  // Serial.printf("RX[%02X] RSSI:%d, RxErr:%d %08X\n",
  //                 RxPkt->Channel, RxPkt->RSSI, RxPacket->RxErr, RxPacket->Packet.HeaderWord);
  // RxPkt->Print(CONS_UART_Write, 1);
  // uint8_t Len=RxPacket->WritePOGNT(Line);
  // if(Len>=8) { Line[Len-1]=0; Serial.println(Line); }
}

static void Radio_RxProcess(void)                                      // process packets in the RxFIFO
{ FSK_RxPacket *RxPkt = RxFIFO.getRead();                              // check for new received packets
  if(RxPkt==0) return;
  LED_Green();                                                         // green flash
  // Serial.printf("Radio_RxProcess() SysID:%d->", RxPkt->SysID);
  RxPkt->DecodeSysID();
  // Serial.printf("%d\n", RxPkt->SysID);
  if(RxPkt->SysID==Radio_SysID_OGN) Radio_RxProcOGN(RxPkt);            // process OGN packet
#if WITH_ADSL
  if(RxPkt->SysID==Radio_SysID_ADSL) Radio_RxProcADSL(RxPkt);          // process ADS-L packet
#endif
  RxFIFO.Read();
  LED_OFF(); }

// ===============================================================================================

static void Sleep(void)                            // shut-down all hardware and go to deep sleep
{ detachInterrupt(USER_KEY);                       // stop user-button interrupt
  // detachInterrupt(GPIO12);                         // stop GPS PPS interrupt
  OLED_ON();
  OLED_Logo();                                     // display logo (for a short time)
  GPS.end();                                       // stop the GPS
  detachInterrupt(RADIO_DIO_1);                    // stop Radio interrupt
  Radio.Sleep();                                   // stop Radio
  delay(500);
  OLED_OFF();                                      // stop OLED
  Display.stop();
  LED_OFF();                                       // stop RGB LED
  Wire.end();                                      // stop I2C
  Serial.end();                                    // stop console UART
  pinMode(Vext, ANALOG);
  pinMode(ADC, ANALOG);
#ifdef WITH_DEBUGPIN
  pinMode(DebugPin, ANALOG);
#endif
  while(1) lowPowerHandler(); }                     // never wake up

/*
static uint32_t Button_PrevSysTime=0;               // [ms] previous sys-time when the Button_Process() was called
static uint32_t Button_PressTime=0;                 // [ms] count the time the button is pressed
static uint32_t Button_IdleTime=0;                  // [ms] count time when the user is not pushing the button

static void Button_Process(void)                    // process the button push/release
{ uint32_t SysTime = millis();                      // [ms]
  uint32_t Diff = SysTime-Button_PrevSysTime;       // [ms] since previous call
  if(!Button_isPressed())                           // if button not pressed (any more)
  { if(Button_PressTime>=100)                       // if was pressed for at least 100ms
    { if(OLED_isON) OLED_NextPage();                // either switch OLED page
               else OLED_ON(); }                    // or turn the OLED back ON
    Button_IdleTime += Diff;                        // [ms] count the idle time
    if(Button_IdleTime>60000) Button_IdleTime=60000; // [ms] top it at 60 sec
    if(Button_IdleTime>=20000) OLED_OFF();          // turn off OLED when idle more than 20sec
    Button_PressTime=0;                             // clear the press-time counter
    Button_LowPower=0; }                            // reset counter to enter sleep
  else                                              // when button is pressed
  { Button_PressTime += Diff;                       // accumulate the press-time
    Button_IdleTime = 0;                            // clear idle time
    if(Button_PressTime>=1000) Button_LowPower=1;   // if more than one second then declare low-power request
  }
  Button_PrevSysTime=SysTime; }
*/

static bool Button_isPressed(void) { return digitalRead(USER_KEY)==0; } // tell if button is being pressed or not at this moment

static bool     Button_LowPower=0;                  // set to one when button pressed for more than one second.

static uint32_t Button_PressTime = 0;               // [ms] when the button was pressed
static  uint8_t Button_ShortPush = 0;               // count short pushes of the button
static  uint8_t Button_LongPush  = 0;               // count longer pushes of the button
static uint32_t Button_PrevSysTime=0;               // [ms] previous sys-time when the Button_Process() was called
static uint32_t Button_IdleTime=0;                  // [ms] count time when the user is not pushing the button

static void Button_Process(void)                    // process the button push/release: as press-release works on interrupts, this can be now called less often
{ uint32_t SysTime = millis();                      // [ms]
  if(Button_isPressed())
  { uint32_t LongPush=SysTime-Button_PressTime;     // [ms] for how long the button pushed already
    // if(LongPush>=1000)
    if(LongPush>=4000) Button_LowPower=1; }         // if more than 3 seconds then request deep sleep
  uint32_t Diff = SysTime-Button_PrevSysTime;       // [ms]
  if(Button_ShortPush==0)
  { Button_IdleTime+=Diff;                          // count idle time
    if(Button_IdleTime>100000) Button_IdleTime=100000; // [ms] top it at 100 sec
    if(Button_IdleTime>=60000 && OLED_Mode==0) OLED_OFF(); }  // turn off OLED when idle more than 60sec
  while(Button_ShortPush)
  { if(!OLED_isON) OLED_ON();
    else                                            // either switch OLED page or the target
    { if(OLED_Mode) OLED_NextTarget();
               else OLED_NextPage(); }
    Button_IdleTime=0;
    Button_ShortPush--; }
#ifdef WITH_TRACKING
  while(Button_LongPush)
  { OLED_Mode = !OLED_Mode;
    if(!OLED_isON) OLED_ON();
    Button_IdleTime=0;
    Button_LongPush--; }
#endif
  Button_PrevSysTime=SysTime; }

static void Button_ForceActive(void)
{ Button_IdleTime=0; OLED_ON(); }                   // activate the OLED, as if the user pushed the button

static void Button_ChangeInt(void)                  // called by hardware interrupt on button push or release
{ if(Button_isPressed())                            // button has been now pressed
  { Button_PressTime=millis(); }                    // [ms] record the moment it as pressed
  else                                              // button has been now released
  { uint32_t Diff = millis()-Button_PressTime;
    if(Diff<100)                                    // it appears something is "pushing" the button at 1sec interval for 12-13ms - could it be battery readout ?
    { // Serial.printf("ButtPulse: %d\n", Diff);
      return; }
    if(Diff<300) Button_ShortPush++;                // if button was pushed shorter than 0.4sec simply increase the counter
    else if(Diff<1000) Button_LongPush++;              // count longer pushes
  }
}

// ===============================================================================================

static void SleepBack(void)                        // go back to deep sleep after an unconfirmed power-on
{ OLED_OFF();                                      // stop OLED
  Display.stop();
  LED_OFF();                                       // stop RGB LED
  Wire.end();                                      // stop I2C
  Serial.end();                                    // stop console UART
  pinMode(Vext, ANALOG);
  pinMode(ADC, ANALOG);
#ifdef WITH_DEBUGPIN
  pinMode(DebugPin, ANALOG);
#endif
  while(1) lowPowerHandler(); }                    // never wake up

void setup()
{ // delay(2000); // prevents USB driver crash on startup, do not omit this

  Random.Word ^= getUniqueID();

  VextON();                               // Turn on power to OLED (and possibly other external devices)
  delay(100);
#ifdef WITH_DEBUGPIN
  DebugPin_Init();
#endif

  ReadBatteryVoltage();

  int8_t Err=Parameters.ReadFromFlash();     // read parameters from Flash
  if(Err<0) { Parameters.setDefault(); Parameters.WriteToFlash(); }
#ifdef HARD_NAME
  strcpy(Parameters.Hard, HARD_NAME);
#endif
#ifdef SOFT_NAME
  strcpy(Parameters.Soft, SOFT_NAME);
#endif

  Serial.begin(Parameters.CONbaud);       // Start console/debug UART
  Serial.println("OGN-Tracker for HELTEC CubeCell with GPS");

  Pixels.begin();                         // Start RGB LED
  Pixels.clear();
  Pixels.setBrightness(0x20);
  LED_Green();
  // Pixels.setPixelColor( 0, 255, 0, 0, 0); // Green
  // Pixels.show();

  pinMode(USER_KEY, INPUT_PULLUP);        // push button

  // OLED_ON();
  Display.init();                                     // Start the OLED
  OLED_isON=1;
  // EEPROM.begin(512);
  if(!Parameters.PowerON)
  { OLED_ConfirmPowerON();
    int Time=2000; int Push=0;
    for( ; Time>0; Time--)
    { delay(1); Push+=digitalRead(USER_KEY)==0;
      if(Push>100) break; }
    if(Time==0) SleepBack();
           else { Parameters.PowerON=1; Parameters.WriteToFlash(); }
  }
  OLED_Logo();

  innerWdtEnable(true);                    // turn on the WDT, can be feed by feedInnerWdt(), has 2.4ssec timeout
                                           // but how to turn it off when going to deep for power-off ?

  attachInterrupt(USER_KEY, Button_ChangeInt, CHANGE);

#ifdef WITH_BMX280
  BMX280_Init();
  if(Baro.ADDR) Serial.printf("BM%280 0x%02X\n", Baro.hasHumidity()?'E':'P', Baro.ADDR);
           else Serial.printf("Neither BMP280 nor BME280 were detected\n");
#endif
#ifdef WITH_BME280
  BME280_Init();
#endif
#ifdef WITH_BMP280
  BMP280_Init();
#endif

  GPS.begin(GPS_BaudRate);
  pinMode(GPIO12, INPUT);                            // GPS PPS and/or LED
  // attachInterrupt(GPIO12, GPS_HardPPS, RISING);      //

  char GPS_Cmd[64];
  uint8_t Len = Format_String(GPS_Cmd, "$PCAS03,1,0,1,1,1,0,0,0");  // send command to enable/disable particular NMEA
  // $PCAS03,nGGA,nGLL,nGSA,nGSV,nRMC,nVTG,nZDA,nTXT // rate to send each NMEA type
  Len += NMEA_AppendCheckCRNL(GPS_Cmd, Len);
  GPS_Cmd[Len]=0;
  Serial1.write(GPS_Cmd);

  Len = Format_String(GPS_Cmd, "$PCAS04,7");  // send command to enable all GNSS systems
  // $PCAS04,Mode      // 1=GPS, 2=BDS, 3=GPS+BDS, 4=GLONASS, 5=GPS+GLONASS, 6=BDS+GLONASS, 7=GPS+BDS+GLONASS
  Len += NMEA_AppendCheckCRNL(GPS_Cmd, Len);
  GPS_Cmd[Len]=0;
  Serial1.write(GPS_Cmd);

  Radio_Init();
  Random.RX  ^= Radio.Random();
  Random.GPS ^= Radio.Random();
  XorShift64(Random.Word);

  // OLED_Info();

#ifdef WITH_DIG_SIGN
  uECC_set_rng(&RNG);
  SignKey.Init();
  SignKey.PrintKeys();
#endif

  PrintPOGNS();               // send to the console (some) current parameters
}

__attribute__((aligned(4)))
static OGN_TxPacket<OGN1_Packet> TxPosPacket, TxStatPacket, TxRelPacket, TxInfoPacket; // OGN position, status and info packets
#ifdef WITH_ADSL
__attribute__((aligned(4)))
static ADSL_Packet ADSL_TxPacket;           // ADS-L packet to be transmitted
#endif
// static uint8_t OGN_Sign[68];                // digital signature to be appended to some position packets
// static uint8_t OGN_SignLen=0;               // digital signature size, 64 + 1 or 2 bytes

// static bool GPS_Done = 0;                   // State: 1 = GPS is sending data, 0 = GPS sent all data, waiting for the next PPS

static uint32_t TxTime0, TxTime1;           // transmision times for the two slots
static OGN_TxPacket<OGN1_Packet> *TxPkt0, *TxPkt1; // OGN packets to transmit in the 1st and 2nd sub-slot
static OGN_TxPacket<OGN1_Packet> *SignTxPkt=0;  // which OGN packet the signature corresponds to
#ifdef WITH_ADSL
static OGN_TxPacket<OGN1_Packet> *ADSL_TxPkt=0; // ADS-L packet to transmit
static bool ADSL_TxSlot=0;                  // transmit ADS-L in the 1st of 2nd sub-slot ?
#endif
#ifdef WITH_FANET
static FANET_Packet FNT_TxPacket;            // FANET packet to transmit
static uint32_t FNT_Freq = 0;                // [Hz] if zero then transmission not scheduled for given slot
static  uint8_t FNT_BackOff=0;               // [sec]
#endif
#ifdef WITH_MESHT
__attribute__((aligned(4)))
static MESHT_Packet MSH_TxPacket;            // Meshtastic packet to transmit
static uint32_t MSH_Freq = 0;                // [Hz] if zero then transmission not scheduled for given slot
static  uint8_t MSH_BackOff=0;               // [sec]
#endif

static int32_t  RxRssiSum=0;                 // [0.5dBm] sum RSSI readouts
static int      RxRssiCount=0;               // count RSSI readouts

static void RxRssiProc(int16_t RSSI) { RxRssiSum+=RSSI*2; RxRssiCount++; } // [dBm]

static int16_t  TxRssiThres=0;               // [dBm] thresdhold for LBT
static uint32_t TxPktCount=0;

static void StartRFslot(void)                // start the TX/RX time slot right after the GPS stops sending data
{ if(RxRssiCount)
  { TxRssiThres = RX_RSSI.getOutput()/2+10;  // add 20dB for the threshold
    // Serial.printf("RxRssi: Thres:%d %d/%d\n", TxRssiThres, RxRssiSum, RxRssiCount);
    RX_RSSI.Process(RxRssiSum/RxRssiCount); RxRssiSum=0; RxRssiCount=0; }

  // Serial.printf("StartRFslot()\n");
  XorShift64(Random.Word);
  GPS_Position &GPS = GPS_Pipe[GPS_Ptr];
  GPS_Satellites = GPS.Satellites;
  GPS_State.TimeValid = GPS.isTimeValid();
  GPS_State.DateValid = GPS.isDateValid();
  GPS_State.FixValid  = GPS.isValid();
  if(GPS_State.TimeValid && GPS_State.DateValid) { LED_Blue(); GPS_PPS_UTC = GPS.getUnixTime(); }    // if time and date are valid
                                           else  { LED_Yellow(); }
  RX_OGN_Count64 += RX_OGN_Packets - RX_OGN_CountDelay.Input(RX_OGN_Packets); // add OGN packets received, subtract packets received 64 seconds ago
  RX_OGN_Packets=0;                                                           // clear the received packet count
  CleanRelayQueue(GPS_PPS_UTC);
  { uint8_t Len=sprintf(Line, "$POGNR,%d,%d,,%+4.1f,,,,%5.3f",
                   Radio_FreqPlan.Plan, RX_OGN_Count64, 0.5*RX_RSSI.getOutput(), 0.001*BattVoltage);
    Len+=NMEA_AppendCheckCRNL(Line, Len);
    Serial.write((const uint8_t *)Line, Len); }
#ifdef WITH_BME280
  BME280_Read(GPS);
#endif
#ifdef WITH_BMP280
  BMP280_Read(GPS);
#endif
  if(GPS.hasBaro)
  { uint8_t Len=0;
    Len+=GPS.WritePGRMZ(Line+Len);
    Len+=GPS.WriteLK8EX1(Line+Len, BattVoltage);
    Serial.write((const uint8_t *)Line, Len); }
  bool TxPos=0;
#ifdef WITH_FANET
  FNT_Freq=0;
#endif
  if(GPS_State.FixValid)                                      // if position is valid
  { GPS_ValidUTC  = GPS.getUnixTime();
    GPS_Altitude  = GPS.Altitude;                             // set global GPS variables
    GPS_Latitude  = GPS.Latitude;
    GPS_Longitude = GPS.Longitude;
    GPS_GeoidSepar= GPS.GeoidSeparation;
    GPS_LatCosine = GPS.LatitudeCosine;
    if(Parameters.FreqPlan==0)
      Radio_FreqPlan.setPlan(GPS_Latitude, GPS_Longitude);      // set Radio frequency plan
    GPS_Random_Update(GPS);
    XorShift64(Random.Word);
    // Serial.printf("Random: %08X:%08X\n", Random.RX, Random.GPS);
#ifdef WITH_MESHT
    MSH_Freq=0;
    if(getMeshtPacket(&MSH_TxPacket, &GPS))                   // produce Meshtastic position packet
    { if(MSH_BackOff) MSH_BackOff--;                          // if successful
      else if(FNT_BackOff && Radio_FreqPlan.Plan<=1) MSH_Freq=Radio_FreqPlan.getFreqOBAND();
    }
    if(MSH_Freq)
    { MSH_TxConfig();
      Radio.SetChannel(MSH_Freq);
      Radio_CAD=(-1);
      Radio.StartCad(4);
      uint8_t Wait=0;
      for( ; Wait<10; Wait++)
      { delay(1);
        // if(Radio.GetStatus()!=RF_CAD) break;
        if(Radio_CAD>=0) break;
        CONS_Proc(); }
      // Serial.printf("MSHcad:%d %dms\n", Radio_CAD, Wait+1);
      if(!Radio_CAD)
      { Radio.Send(MSH_TxPacket.Byte, MSH_TxPacket.Len);  // this call takes about 3ms but it only triggers the transmission
        MSH_BackOff = 20 + Random.RX%21; }
      MSH_Freq=0; }
#endif
#ifdef WITH_FANET
    if(getFNTpacket(FNT_TxPacket, GPS))                       // produce FANET position packet
    { if(FNT_BackOff) FNT_BackOff--;                          // if successful (within altitude limit)
      else if(Radio_FreqPlan.Plan<=1) FNT_Freq=Radio_FreqPlan.getFreqFANET();
    }
    if(FNT_Freq)
    { FNT_TxConfig();
      Radio.SetChannel(FNT_Freq);
      Radio_CAD=(-1);
      Radio.StartCad(4);
      uint8_t Wait=0;
      for( ; Wait<10; Wait++)
      { delay(1);
        // if(Radio.GetStatus()!=RF_CAD) break;
        if(Radio_CAD>=0) break;
        CONS_Proc(); }
      // Serial.printf("FNTcad:%d %dms\n", Radio_CAD, Wait+1);
      if(!Radio_CAD)
      { Radio.Send(FNT_TxPacket.Byte, FNT_TxPacket.Len);
        FNT_BackOff = 9 + Random.RX%3; }
      FNT_Freq=0; }
#endif
    getPosPacket(TxPosPacket.Packet, GPS);                    // produce position packet to be transmitted
#ifdef WITH_DIG_SIGN
    if(SignKey.KeysReady)
    { SignKey.Hash(GPS_PPS_UTC, TxPosPacket.Packet.Byte(), TxPosPacket.Packet.Bytes); // produce SHA256 hash (takes below 1ms)
      SignTxPkt = &TxPosPacket; }
#endif
    TxPosPacket.Packet.Whiten();
    TxPosPacket.calcFEC();                                    // position packet is ready for transmission
#ifdef WITH_ADSL
    if(Radio_FreqPlan.Plan<=1)
    { ADSL_TxPkt = &TxPosPacket;
      getAdslPacket(ADSL_TxPacket, GPS);
      ADSL_TxPacket.Scramble();                              // this call hangs when -Os is used to compile
      ADSL_TxPacket.setCRC();
      ADSL_TxSlot = Random.GPS&0x20; }
    else ADSL_TxPkt=0;
#endif
    TxPos=1; }
  // Serial.printf("StartRFslot() #1\n");
  CONS_Proc();
  ReadBatteryVoltage();
  CONS_Proc();
  OLED_DispPage(GPS);                                         // display GPS data or other page on the OLED
  CONS_Proc();
  // Serial.printf("StartRFslot() #2\n");
  uint8_t Wait=50;                    // [ms]
  for( ; Wait>0; Wait--)              // wait for FANET/MESHT transmission to complete
  { if(!Radio_TxRunning()) break;
    delay(1); }
  Radio_Slot=0;
  Radio_SysID=Radio_SysID_OGN_ADSL;      // receive OGN and ADS-L in parallel
  Radio_Channel=Radio_FreqPlan.getChannel(GPS_PPS_UTC, Radio_Slot, 1);
  Radio_TxConfig(Radio_SysID);
  Radio.SetChannel(Radio_FreqPlan.getChanFrequency(Radio_Channel));
  Radio_RxConfig(Radio_SysID);
  // Serial.printf("RFslot Sys:%d Chan:%d TxPos:%d RssiThres:%+d TxPkt:%d\n",
  //            Radio_SysID, Radio_Channel, TxPos, TxRssiThres, TxPktCount);
  Radio.RxBoosted(0);
  XorShift64(Random.Word);
  TxTime0 = Random.RX  % 97;                                 // transmit times within slots
  TxTime1 = Random.GPS % 99;
  TxPkt0=TxPkt1=0;
  if(TxPos) TxPkt0 = TxPkt1 = &TxPosPacket;
  XorShift64(Random.Word);
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
      if(Random.RX&0x10) TxPkt1 = &TxInfoPacket;                // put it randomly into 1st or 2nd time slot
                    else TxPkt0 = &TxInfoPacket;
      InfoTxBackOff = 15 + (Random.RX%3);                       // 16+/-1
    }
  }
  XorShift64(Random.Word);
  static uint8_t RelayTxBackOff=0;
  if(RelayTxBackOff) RelayTxBackOff--;
  else if(GetRelayPacket(&TxRelPacket))
  { if(Random.RX&0x20) TxPkt1 = &TxRelPacket;
                  else TxPkt0 = &TxRelPacket;
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
      TxTime1 = 200 + (TxTime0/2);
      SignTxBackOff = 6 + (Random.RX%7); }
  }
#endif
  TxTime0 += 400;
  TxTime1 += 800;
  LED_OFF();
}

static void PPS_SoftEdge(uint32_t msTime, uint32_t msDelay)
{ uint32_t msDiff = msTime-GPS_PPS_ms; if(msDiff<=800) return;
  GPS_PPS_UTC++; GPS_PPS_ms=msTime-msDelay;
  // printf("SoftPPS: %10d:%10d\r\n", GPS_PPS_UTC, GPS_PPS_ms);
}

static void PPS_HardEdge(uint32_t msTime)
{ GPS_PPS_UTC++; GPS_PPS_ms=msTime;
  // printf("HardPPS: %10d:%10d\r\n", GPS_PPS_UTC, GPS_PPS_ms);
}

static void PPS_Process(void)
{ static bool PrevPPS=0;
  static uint32_t PrevTime=0;
  bool PPS = GPS_ReadPPS();
  if(PPS!=PrevPPS)                                               // if PPS line changed state
  { if(PPS)                                                      // if positive edge (ignore negative edge)
    { uint32_t Time=millis();                                    // [ms]
      uint32_t DiffTime = Time-PrevTime;                         // [ms]
      // Serial.printf("PPS: %4d\n", DiffTime);
      if(DiffTime>=990 && DiffTime<=1010) PPS_HardEdge(Time);    // [ms] must be 1000+/-10ms
      PrevTime=Time; }
    PrevPPS=PPS; }
}

void loop()
{ // CY_PM_WFI;                                                      // sleep, while waiting for an interrupt (reduces power consumption ?)
  delay(1);                                                       //

  PPS_Process();                                                  // check for PPS edge
  Button_Process();                                               // check for button short/long press
  if(Button_LowPower) { Parameters.PowerON=0; Parameters.WriteToFlash(); Sleep(); return; }

  Radio_RxProcess();                                              // process received packets, if any

  CONS_Proc();                                                    // process input from the console
  if(GPS_Process()==0) { GPS_Idle++; }                            // process input from the GPS
                  else { GPS_Idle=0; RxRssiProc(Radio.Rssi(MODEM_FSK)); } // [0.5dBm]
  if(GPS_State.BurstDone)                                         // if state is GPS not sending data
  { if(GPS_Idle<2)                                                // GPS (re)started sending data
    { GPS_State.BurstDone=0;                                      // change the state to GPS is sending data
      PPS_SoftEdge(millis(), GPS_State.FixValid?50:30);
      // GPS_PPS_ms = millis()-(GPS_State.FixValid?50:30); // Parameters.PPSdelay;                // record the est. PPS time
      // printf("GPS slot: start: %d [ms]\n\r", millis());
      // GPS_PPS_UTC++;
    }
  }
  else
  { if(GPS_Idle>10)                                               // GPS stopped sending data
    { // printf("GPS slot stop: %d [ms]\n\r", millis());
      GPS_SatMon.Sort();
      GPS_SatCnt=GPS_SatMon.CalcStats(GPS_SatSNR);
      StartRFslot();                                              // start the next RF slot
      if(GPS_PPS_UTC%10==0)
      { GPS_SatMon.PrintStats(Line);
        Serial.println(Line); }
      GPS_State.BurstDone=1; }
  }

  uint32_t SysTime = millis() - GPS_PPS_ms;
  if(Radio_Slot==0)                                             // while in the 1st sub-slot
  { if(TxPkt0 && SysTime>=TxTime0 && !Radio_TxRunning())        //
    { int16_t RxRssi=Radio.Rssi(MODEM_FSK); RxRssiProc(RxRssi); // [dBm]
      if(RxRssi<=TxRssiThres)
      { int TxLen=0; // Serial.printf("1\n");
#ifdef WITH_ADSL
        if(ADSL_TxPkt==TxPkt0 && ADSL_TxSlot==0)
        { TxLen=ADSL_ManchTx(ADSL_TxPacket); TxPktCount++; }
        else
#endif
        { TxLen=OGN_ManchTx(*TxPkt0); TxPktCount++; }
        // Serial.printf("TX[0]:%4dms %08X [%d:%d] [%2d]\n",
        //          SysTime, TxPkt0->Packet.HeaderWord, SignKey.SignReady, SignTxPkt==TxPkt0, TxLen);
        TxPkt0=0; }
      else
      { // Serial.printf("_");
        TxTime0 += 10+Random.RX%47; }
    }
    if(SysTime >= 800)                                      // if 800ms from PPS then switch to the 2nd sub-slot
    { Radio_Slot=1;
      Radio_TxConfig(Radio_SysID);
      Radio_RxConfig(Radio_SysID);
      Radio_Channel=Radio_FreqPlan.getChannel(GPS_PPS_UTC, Radio_Slot, 1);
      Radio.SetChannel(Radio_FreqPlan.getChanFrequency(Radio_Channel));
      Radio.RxBoosted(0);
      // Serial.printf("Slot #1: %d\r\n", SysTime);
    }
  } else                                                          // while in the 2nd sub-slot
  { if(TxPkt1 && SysTime >= TxTime1 && !Radio_TxRunning())
    { int16_t RxRssi=Radio.Rssi(MODEM_FSK); RxRssiProc(RxRssi); // [dBm]
      if(RxRssi<=TxRssiThres)
      { int TxLen=0; // Serial.printf("2\n");
#ifdef WITH_ADSL
        if(ADSL_TxPkt==TxPkt1 && ADSL_TxSlot==1)
        { TxLen=ADSL_ManchTx(ADSL_TxPacket); TxPktCount++; }
        else
#endif
        { TxLen=OGN_ManchTx(*TxPkt1); TxPktCount++; }
        // Serial.printf("TX[1]:%4dms %08X [%d:%d] [%2d]\n",
        //          SysTime, TxPkt1->Packet.HeaderWord, SignKey.SignReady, SignTxPkt==TxPkt1, TxLen);
        TxPkt1=0; }
      else
      { // Serial.printf("-");
        TxTime1 += 10+Random.RX%47; }
    }
  }

}

