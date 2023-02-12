#ifndef __FANET_H__
#define __FANET_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "format.h"
#include "intmath.h"

// ===============================================================================================

class FANET_Packet
{ public:
   union
   { uint8_t Flags;
     struct
     { uint8_t  CR:3;  // Coding rate used (RX) or to be used (TX)
       bool hasCRC:1;  // CRC was there (RX)
       bool badCRC:1;  // CRC was bad (RX)
       bool   Done:1;
     } ;
   } ;
   uint8_t Len;       // [bytes] packet length
   static const int MaxBytes = 40;
   uint8_t Byte[MaxBytes+2];

  public:
   FANET_Packet() { Len=0; }

   uint8_t Dump(char *Out)
   { uint8_t Len=0;
     for(int Idx=0; Idx<this->Len; Idx++)
     { Len+=Format_Hex(Out+Len, Byte[Idx]); }
     return Len; }

   bool ExtHeader(void) const { return Byte[0]&0x80; }      // is there extended header ?
   bool Forward(void) const { return Byte[0]&0x40; }        // forward flag
   uint8_t Type(void) const { return Byte[0]&0x3F; }        // message type
   uint32_t getAddr(void) const { uint32_t Addr=Byte[1]; Addr<<=8; Addr|=Byte[3]; Addr<<=8; Addr|=Byte[2]; return Addr; } // 24-bit source address
   uint8_t getAddrPref(void) const { return Byte[1]; }

   uint8_t getAddrType(void) const                          // address-type based on prefix
   { uint8_t Pref=getAddrPref();
     if(Pref==0x08 || Pref==0x11 || Pref==0x20 || Pref==0xDD || Pref==0xDE || Pref==0xDF) return 2;
     return 3; }

   void setAddress(uint32_t Addr) { setAddrPref(Addr>>16); setAddrLow(Addr); }   // full 24-bit address
   void setAddrPref(uint8_t Prefix) { Byte[1]=Prefix; }                          // address prefix
   void setAddrLow(uint16_t Addr  ) { Byte[2]=Addr; Byte[3]=Addr>>8; }           // lower 16-bits of the address
   void setHeader(uint8_t Type) { Byte[0] = 0x40 | (Type&0x3F); }
   void setType(uint8_t Type) { Byte[0] = (Byte[0]&0xC0) | (Type&0x3F); }        // packet-type: 1=air-position

   uint8_t ExtHeaderLen(void) const // length ot the extended header (zero in most cases)
   { if(!ExtHeader()) return 0;
     uint8_t Len=1;
     if(Byte[4]&0x20) Len+=3;    // if Unicast
     if(Byte[4]&0x10) Len+=4;    // if Signature
     return Len; }

  uint8_t MsgOfs(void) const { return 4+ExtHeaderLen(); }              // offset to the actual message (past the header and ext. header)
  uint8_t MsgLen(void) const { return Len-4-ExtHeaderLen(); }          // length of the actual message
  const uint8_t *Msg(void) const { return Byte+MsgOfs(); }             // pointer to the message, past the header
  uint8_t *Msg(void)         { return Byte+MsgOfs(); }

  void setName(const char *Name)
  { setHeader(2);
    uint8_t Ofs;
    for(Ofs=MsgOfs(); Ofs<MaxBytes; Ofs++)
    { char ch = *Name++; if(ch==0) break;
      Byte[Ofs]=ch; }
    // xif(Ofs<MaxBytes) Byte[Ofs++]=0;
    Len=Ofs; }

  static float FloatCoord(int32_t Coord)                               // convert from FANET cordic units to float degrees
  { // const float Conv = 90.0/0x40000000;                                // conversion factor (exact cordic)
    const float Conv = 90.0007295677/0x40000000;                       // FANET cordic conversion factor (not exactly cordic)
    return Conv*Coord; }

  static int32_t getLat(const uint8_t *Byte)                           // FANET cordic units
  { int32_t Latitude=Byte[2]; Latitude<<=8; Latitude|=Byte[1]; Latitude<<=8; Latitude|=Byte[0]; Latitude<<=7; return Latitude; }
  static void setLat(uint8_t *Byte, int32_t Lat)
  { Lat = (Lat+0x40)>>7; Byte[0]=Lat; Byte[1]=Lat>>8; Byte[2]=Lat>>16; }
  static int32_t getLon(const uint8_t *Byte)                           // FANET cordic units
  { int32_t Longitude=Byte[2]; Longitude<<=8; Longitude|=Byte[1]; Longitude<<=8; Longitude|=Byte[0]; Longitude<<=8; return Longitude; }
  static void setLon(uint8_t *Byte, int32_t Lon)
  { Lon = (Lon+0x80)>>8; Byte[0]=Lon; Byte[1]=Lon>>8; Byte[2]=Lon>>16; }

  static uint16_t getSpeed(uint8_t Byte)                               // [0.5km/h] for Type=1
  { if(Byte<0x80) return Byte;
    return (uint16_t)5*(Byte-0x80); }
  static void setSpeed(uint8_t *Byte, uint16_t Speed)
  { if(Speed>=128) { Speed=(Speed+2)/5; if(Speed>=0x80) Speed=0x7F; Speed|=0x80; }
    Byte[0]=Speed; }

  static int16_t getClimb(uint8_t Byte)                                // [0.1m/s]
  { int16_t Climb = Byte&0x3F; if(Byte&0x40) Climb|=0xFFC0;
    if(Byte&0x80) return Climb*5;
    return Climb; }
  static void setClimb(uint8_t *Byte, int16_t Climb)                   // [0.1m/s]
  { if(Climb>63)
    { Climb = (Climb+2)/5; if(Climb>63) Climb=63;
      Byte[0] = 0x80 | Climb; }
    else if(Climb<(-63))
    { Climb = (Climb-2)/5; if(Climb<(-63)) Climb=(-63);
      Byte[0] = 0x80 | Climb; }
    else
    { Byte[0] = Climb&0x7F; }
  }

  static uint16_t getDir(uint8_t Byte)                                 // [deg]
  { uint16_t Dir = ((uint16_t)45*Byte+0x10)>>5; return Dir; }

  static uint16_t getPressure(const uint8_t *Byte)                     // [0.1hPa]
  { uint16_t Press = Byte[1]; Press<<=8; Press|=Byte[0]; return Press+4300; }

  static int16_t getTurnRate(uint8_t Byte)                             // [0.25deg/s]
  { int16_t Rate = Byte&0x7F; if(Byte&0x40) Rate|=0xFF80;
    if(Byte&0x80) return Rate<<2;
    return Rate; }
  static void setTurnRate(uint8_t *Byte, int16_t Rate)
  { if(Rate>= 64 ) { if(Rate>  254 ) Rate=  254 ; Byte[0] = 0x80 |  ((Rate+2)>>2);       return; }
    if(Rate<(-64)) { if(Rate<(-254)) Rate=(-254); Byte[0] = 0x80 | (((Rate+2)>>2)&0x7F); return; }
    Byte[0] = Rate&0x7F; }

  static int16_t getQNE(uint8_t Byte)                                  // [m] difference between pressure altitude and GPS altitude
  { int16_t QNE = Byte&0x7F; if(Byte&0x40) QNE|=0xFF80;
    if(Byte&0x80) return QNE<<2;
    return QNE; }
  static void setQNE(uint8_t *Byte, int16_t QNE)
  { if(QNE>= 64 ) { if(QNE>  254 ) QNE=  254 ; Byte[0] = 0x80 |  ((QNE+2)>>2);       return; }
    if(QNE<(-64)) { if(QNE<(-254)) QNE=(-254); Byte[0] = 0x80 | (((QNE+2)>>2)&0x7F); return; }
    Byte[0] = QNE&0x7F; }

  static uint16_t getAltitude(const uint8_t *Byte)                     // [m]
  { uint16_t Alt = Byte[1]; Alt<<=8; Alt|=Byte[0]; Alt&=0x0FFF;
    if(Alt<0x800) return Alt;
    return (Alt-0x800)<<2; }
  void setAltitude(uint8_t *Byte, uint16_t Alt)                        // [m]
  { if(Alt>=0x800) { Alt>>=2; if(Alt>0x800) Alt=0x800; Alt|=0x800; }
    Byte[0]=Alt; Byte[1] = (Byte[1]&0xF0) | (Alt>>8); }

  //                       [0..7]         [0..1] [FANET cordic] [FANET cordic]     [m]     [cordic]        [0.1m/s]       [0.1m/s]    [0.1deg/s]
  void setAirPos(uint8_t AcftType, uint8_t Track, int32_t Lat, int32_t Lon, int16_t Alt, uint8_t Dir, uint16_t Speed, int16_t Climb, int16_t Turn)
  { setHeader(1);
    uint8_t Ofs=MsgOfs();
    Len=Ofs+12;
    setLat(Byte+Ofs, Lat);            // [cordic]
    setLon(Byte+Ofs+3, Lon);          // [cordic]
    Byte[Ofs+7]=(AcftType<<4) | (Track<<7);
    if(Alt<0) Alt=0;
    setAltitude(Byte+Ofs+6, Alt);     // [m]
    Speed = (Speed*23+16)>>5;         // [0.1m/s] => [0.5km/h]
    setSpeed(Byte+Ofs+8, Speed);      // [0.5km/h]
    setClimb(Byte+Ofs+9, Climb);      // [0.1m/s]
    Byte[Ofs+10] = Dir;               // [cordic]
    setTurnRate(Byte+Ofs+11, Turn*2/5); } // [0.25deg/s]

  void setQNE(int32_t StdAltitude)    // [m] only for air-position
  { uint8_t Ofs=MsgOfs();
    int32_t Alt=getAltitude(Byte+Ofs+6);
    if(Len<(Ofs+13)) Len=Ofs+13;
    setQNE(Byte+Ofs+12, StdAltitude-Alt); }

  //                    [0..15]         [0..1]  [FANET cordic]  [FANET cordic]
  void setGndPos(uint8_t Status, uint8_t Track, int32_t Lat, int32_t Lon)
  { setHeader(7);
    uint8_t Ofs=MsgOfs();
    Len=11;
    setLat(Byte+Ofs, Lat);
    setLon(Byte+Ofs+3, Lon);
    Byte[Ofs+6] = (Status<<4) | Track; }

/*
 * $FNNGB,manufacturer(hex),id(hex),name(up to 32bytes),type/status,latitude,longitude,altitude,climb,speed,heading*checksum
 * manufacturer:	1-2 chars hex
 * id:			1-4 chars hex
 * name:		string up to 32 chars
 * type/status:		while airborne: aircraft type: 0-7 (3D tracking), else: status: 0-15 (2D tracking) +10 -> 10-25
 * latitude:		%.5f in degree
 * longitude:		%.5f in degree
 * altitude:		%.f in meter, -1000 for ground
 * climb:		%.1f in m/s
 * speed:		%.1f in km/h
 * heading:		%.f in degree
 *
 * for the types please see: https://github.com/3s1d/fanet-stm32/blob/master/Src/fanet/radio/protocol.txt
 *
 */

  uint8_t WriteFNNGB(char *Out)
  { return 0; }

  int DecodePosition(float &Lat, float &Lon, int &Alt)
  { uint8_t Idx=MsgOfs();
    if(Type()==1)
    { Lat = FloatCoord(getLat(Byte+Idx));
      Lon = FloatCoord(getLon(Byte+Idx+3));
      Alt = getAltitude(Byte+Idx+6);
      return 3; }
    if(Type()==7)
    { Lat = FloatCoord(getLat(Byte+Idx));
      Lon = FloatCoord(getLon(Byte+Idx+3));
      Alt = 0;
      return 2; }
    return 0; }

  void Print(const char *Name=0) const
  { if(Name) printf("%s ", Name);
    printf("[%2d:%d:%2d] FNT%06X", Len, Type(), MsgLen(), getAddr());
    if(Type()==2)                                                      // Name
    { printf(" ");
      for(uint8_t Idx=MsgOfs(); Idx<Len; Idx++)
        printf("%c", Byte[Idx]);
      printf("\n"); return; }
    if(Type()==3)                                                      // Message
    { uint8_t Idx=MsgOfs();
      printf(" Msg%02X: ", Byte[Idx++]);
      for( ; Idx<Len; Idx++)
        printf("%c", Byte[Idx]);
      printf("\n"); return; }
    if(Type()==4)                                                    // Service, mostly meteo
    { uint8_t Idx=MsgOfs(); uint8_t Service=Byte[Idx++];
      int32_t Lat=getLat(Byte+Idx); Idx+=3;                          // [FANET cordic]
      int32_t Lon=getLon(Byte+Idx); Idx+=3;                          // [FANET cordic]
      printf(" [%+09.5f,%+010.5f] s%02X", FloatCoord(Lat), FloatCoord(Lon), Service);
      if(Service&0x80) printf(" Igw");
      if(Service&0x40) printf(" %+3.1fC", 0.5*(int8_t)Byte[Idx++]);  // temperature
      if(Service&0x20)                                               // wind direction and speed
      { uint16_t Dir  = Byte[Idx++];                                 // [cordic]
        uint16_t Wind = getSpeed(Byte[Idx++]);                       // [0.2km/h]
        uint16_t Gust = getSpeed(Byte[Idx++]);                       // [0.2km/h]
        printf(" %03.0fdeg %3.1f/%3.1fkm/h", (180.0/128)*Dir, 0.2*Wind, 0.2*Gust); }
      if(Service&0x10) printf(" %3.1f%%", 0.4*Byte[Idx++]);          // humidity
      if(Service&0x08)                                               // pressure
      { printf(" %3.1fhPa", 0.1*getPressure(Byte+Idx)); Idx+=2; }
      if(Service&0x02) printf(" %1.0f%%", (100.0/15)*Byte[Idx++]);   // charge state
      printf("\n"); return; }
    if(Type()==1)                                                    // airborne position
    { uint8_t Idx=MsgOfs(); uint8_t AcftType=Byte[Idx+7]>>4;
      int32_t Lat=getLat(Byte+Idx);                                  // [FANET cordic]
      int32_t Lon=getLon(Byte+Idx+3);                                // [FANET cordic]
      uint16_t Alt=getAltitude(Byte+Idx+6);                          // [m]
      uint16_t Speed=getSpeed(Byte[Idx+8]);                          // [0.5km/h]
       int16_t Climb=getClimb(Byte[Idx+9]);                          // [0.1m/s]
      printf(" [%+09.5f,%+010.5f] %dm %3.1fkm/h %03.0fdeg %+4.1fm/s a%X%c",
          FloatCoord(Lat), FloatCoord(Lon), Alt, 0.5*Speed, (180.0/128)*Byte[Idx+10], 0.1*Climb,
          AcftType&0x07, AcftType&0x08?'T':'H');
      if((Idx+11)<Len)
      { int16_t Rate = getTurnRate(Byte[Idx+11]);
        printf(" %+3.1fdeg/s", 0.25*Rate); }
      if((Idx+12)<Len)
      { int16_t QNE = getQNE(Byte[Idx+12]);
        printf(" %+dm", QNE); }
      printf("\n"); return; }
    if(Type()==7)                                                    // ground position
    { uint8_t Idx=MsgOfs(); uint8_t Status=Byte[Idx+6];
      int32_t Lat=getLat(Byte+Idx);                                  // [FANET cordic]
      int32_t Lon=getLon(Byte+Idx+3);                                // [FANET cordic]
      printf(" [%+09.5f,%+010.5f] s%02X", FloatCoord(Lat), FloatCoord(Lon), Status);
      printf("\n"); return; }
    if(Type()==8)                                                    // Hardware/Software
    { uint8_t Idx=MsgOfs();
      uint8_t Hw = Byte[Idx];
      uint16_t Fw = Byte[Idx+2]; Fw<<=8; Fw|=Byte[Idx+1];
      printf(" Hw%02X Fw%02d.%02d.%04d%c", Hw, Fw&0x1F, (Fw>>5)&0x0F, 2019+((Fw>>9)&0x3F), Fw&0x8000?'d':'r');
      printf("\n"); return; }
    printf("\n");
  }

  uint8_t Read(const char *Inp)                                      // read packet from a hex dump
  { for( Len=0; Len<MaxBytes; Len++)                                 // read as many hex bytes as you can
    { int8_t Upp = Read_Hex1(Inp[0]); if(Upp<0) break;               // 1st digit
      int8_t Low = Read_Hex1(Inp[1]); if(Low<0) break;               // 2nd digit
      Byte[Len] = (Upp<<4) | Low; Inp+=2; }                          // new byte, count input
    return Len; }                                                    // return number of bytes read = packet length

   static int32_t CoordUBX(int32_t Coord) { return ((int64_t)900007296*Coord+0x20000000)>>30; } // convert FANET-cordic to UBX 1e-7deg units
                                                // ((int64_t)900000000*Coord+0x20000000)>>30;   // this is the exact formula, but FANET is not exact here

   static int Format_Lat(char *Str, int32_t Lat, char &HighRes) // format latitude after APRS
   { Lat = CoordUBX(Lat);                                       // convert from FANET cordic to UBX 1e-7 deg
     char Sign;
     if(Lat>0) { Sign='N'; }
          else { Sign='S'; Lat=(-Lat); }
     int32_t Dig;
     Dig=Lat/100000000; (*Str++)='0'+Dig; Lat-=Dig*100000000;
     Dig=Lat/10000000;  (*Str++)='0'+Dig; Lat-=Dig*10000000;
     Lat*=60;
     Dig=Lat/100000000; (*Str++)='0'+Dig; Lat-=Dig*100000000;
     Dig=Lat/10000000;  (*Str++)='0'+Dig; Lat-=Dig*10000000;
     (*Str++)='.';
     Dig=Lat/1000000;   (*Str++)='0'+Dig; Lat-=Dig*1000000;
     Dig=Lat/100000;    (*Str++)='0'+Dig; Lat-=Dig*100000;
     Dig=Lat/10000;     HighRes ='0'+Dig; Lat-=Dig*10000;
     (*Str++)=Sign;
     return 8; }

   static int Format_Lon(char *Str, int32_t Lon, char &HighRes) // format longitude after APRS
   { Lon = CoordUBX(Lon);                                       // convert from FANET cordic to UBX 1e-7 deg
     char Sign;
     if(Lon>0) { Sign='E'; }
          else { Sign='W'; Lon=(-Lon); }
     int32_t Dig;
     Dig=Lon/1000000000; (*Str++)='0'+Dig; Lon-=Dig*1000000000;
     Dig=Lon/100000000;  (*Str++)='0'+Dig; Lon-=Dig*100000000;
     Dig=Lon/10000000;   (*Str++)='0'+Dig; Lon-=Dig*10000000;
     Lon*=60;
     Dig=Lon/100000000;  (*Str++)='0'+Dig; Lon-=Dig*100000000;
     Dig=Lon/10000000;   (*Str++)='0'+Dig; Lon-=Dig*10000000;
     (*Str++)='.';
     Dig=Lon/1000000;    (*Str++)='0'+Dig; Lon-=Dig*1000000;
     Dig=Lon/100000;     (*Str++)='0'+Dig; Lon-=Dig*100000;
     Dig=Lon/10000;      HighRes ='0'+Dig; Lon-=Dig*10000;
     (*Str++)=Sign;
     return 9; }

} ;

class FANET_RxPacket: public FANET_Packet
{ public:
   uint32_t  sTime;         // [ s] reception time
   uint16_t msTime;         // [ms]
   int16_t FreqOfs;         // [ 10Hz]
    int8_t SNR;             // [0.25dB]
    int8_t RSSI;            // [dBm]
   uint8_t BitErr;          // number of bit errors
   uint8_t CodeErr;         // number of block errors
   uint8_t Sync;            // sync symbols used: 0xF1 for sx127x but 0x12 for sx1262 ?

  public:
   void setTime(double RxTime) { sTime=floor(RxTime); msTime=floor(1000.0*(RxTime-sTime)); }
   double getTime(void) const { return (double)sTime+0.001*msTime; }
   uint32_t SlotTime(void) const { uint32_t Slot=sTime; if(msTime<100) Slot--; return Slot; }

   void Print(char *Name=0) const
   { char HHMMSS[8];
     Format_HHMMSS(HHMMSS, SlotTime());  HHMMSS[6]='h'; HHMMSS[7]=0;
     printf("%s CR%c%c%c %3.1fdB/%de %+3.1fkHz ", HHMMSS, '0'+CR, hasCRC?'c':'_', badCRC?'-':'+', 0.25*SNR, BitErr, 1e-2*FreqOfs);
     FANET_Packet::Print(Name); }

   int PrintJSON(char *JSON, uint8_t AddrType=0) const
   { const uint8_t *Msg = this->Msg();
     uint8_t MsgLen = this->MsgLen();
     uint8_t Type = this->Type();
     if(Type!=1 && Type!=7) { JSON[0]=0; return 0; }
     int Len=0;
     JSON[Len++]='{';
     uint32_t Address = getAddr();
     if(AddrType==0) AddrType = getAddrType();
     Len+=Format_String(JSON+Len, "\"Address\":\"");
     Len+=Format_Hex(JSON+Len, Byte[1]);
     Len+=Format_Hex(JSON+Len, Byte[3]);
     Len+=Format_Hex(JSON+Len, Byte[2]);
     Len+=Format_String(JSON+Len, "\", \"AddrType\":");
     JSON[Len++] = '0'+AddrType;
     Len+=Format_String(JSON+Len, "\", \"ID\":\"");
     Len+=Format_Hex(JSON+Len, Address | ((uint32_t)AddrType<<24));
     uint32_t Time = SlotTime(); // sTime; if(msTime<100) Time--;
     Len+=Format_String(JSON+Len, "\", \"Time\":");
     Len+=Format_UnsDec(JSON+Len, Time);
     if(Type==1)
     { const uint8_t OGNtype[8] = {   0,    7,    6,  0xB,    1,     8,    3,  0xD } ; // OGN aircraft types
       uint8_t AcftType=Msg[7]>>4;                             // get the aircraft-type and online-track flag
       Len+=Format_String(JSON+Len, ", \"AcftType\":");
       Len+=Format_UnsDec(JSON+Len, OGNtype[AcftType&0x7]);
       uint32_t Alt=getAltitude(Msg+6);                        // [m] decode the altitude
       uint32_t Speed=getSpeed(Msg[8]);                        // [0.5km/h] ground speed
       Speed = (Speed*355+0x80)>>8;                            // [0.5km/h] => [0.1m/s] convert
       int32_t Climb=getClimb(Msg[9]);                         // [0.1m/s] climb rate
       uint16_t Dir=getDir(Msg[10]);                           // [deg]
       Len+=Format_String(JSON+Len, ", \"Alt\":");
       Len+=Format_UnsDec(JSON+Len, Alt);
       Len+=Format_String(JSON+Len, ", \"Track\":");
       Len+=Format_UnsDec(JSON+Len, Dir);
       Len+=Format_String(JSON+Len, ", \"Speed\":");
       Len+=Format_UnsDec(JSON+Len, Speed, 2, 1);
       Len+=Format_String(JSON+Len, ", \"Climb\":");
       Len+=Format_SignDec(JSON+Len, Climb, 2, 1, 1);
       if(MsgLen>11)
       { int16_t Turn=getTurnRate(Msg[11]);
         Len+=Format_String(JSON+Len, ", \"Turn\":");
         Len+=Format_SignDec(JSON+Len, Turn*10/4, 2, 1, 1); }
       if(MsgLen>12)
       { int32_t AltStd=Alt; Alt+=getQNE(Msg[12]);
         Len+=Format_String(JSON+Len, ", \"StdAlt\":");
         Len+=Format_SignDec(JSON+Len, AltStd, 1, 0, 1); }
     }
     if(Type==1 || Type==7)
     { int32_t Lat = getLat(Msg);                              // [cordic] decode the latitude
       int32_t Lon = getLon(Msg+3);                            // [cordic] decode the longitude
       Len+=Format_String(JSON+Len, ", \"Lat\":");
       Len+=Format_SignDec(JSON+Len, CoordUBX(Lat), 8, 7, 1);
       Len+=Format_String(JSON+Len, ", \"Lon\":");
       Len+=Format_SignDec(JSON+Len, CoordUBX(Lon), 8, 7, 1); }
     Len+=Format_String(JSON+Len, ", \"RxProt\":\"FNT\"");
     if(SNR>0)
     { Len+=Format_String(JSON+Len, ", \"RxSNR\":");
       Len+=Format_SignDec(JSON+Len, ((int16_t)SNR*10+2-843)>>2, 2, 1, 1); }
     Len+=Format_String(JSON+Len, ", \"RxErr\":");
     Len+=Format_UnsDec(JSON+Len, BitErr);
     Len+=Format_String(JSON+Len, ", \"RxFreqOfs\":");
     Len+=Format_SignDec(JSON+Len, FreqOfs/10, 1, 1);
     JSON[Len++]=' '; JSON[Len++]='}';
     JSON[Len]=0; return Len; }

   int WriteStxJSON(char *JSON, uint8_t AddrType=0) const
   { int Len=0;
     Len+=Format_String(JSON+Len, "\"addr\":\"");
     Len+=Format_Hex(JSON+Len, Byte[1]);
     Len+=Format_Hex(JSON+Len, Byte[3]);
     Len+=Format_Hex(JSON+Len, Byte[2]);
     JSON[Len++]='\"';
     JSON[Len++]=',';
     Len+=Format_String(JSON+Len, "\"addr_type\":");
     if(AddrType==0) AddrType = getAddrType();
     JSON[Len++] = '0'+AddrType;
     const uint8_t *Msg = this->Msg();
     uint8_t  MsgLen = this->MsgLen();
     uint8_t Type = this->Type();
     uint32_t Time = SlotTime(); // sTime; if(msTime<100) Time--;
     Len+=Format_String(JSON+Len, ",\"time\":");
     Len+=Format_UnsDec(JSON+Len, Time);
     int64_t RxTime=(int64_t)sTime-Time; RxTime*=1000; RxTime+=msTime;
     Len+=Format_String(JSON+Len, ",\"rx_time\":");
     Len+=Format_SignDec(JSON+Len, RxTime, 4, 3, 1);
     if(Type==2)
     { Len+=Format_String(JSON+Len, ",\"Name\":\"");
       for(int Idx=0; Idx<MsgLen; Idx++)
       { uint8_t Byte=Msg[Idx]; if(Byte==0) break;
         JSON[Len++]=Byte; }
       JSON[Len++]='\"'; }
     if(Type==1)                                               // for airborne position
     { uint8_t AcftType=Msg[7]>>4;                             // get the aircraft-type and online-track flag
       Len+=Format_String(JSON+Len, ",\"acft_type\":\"");
       JSON[Len++] = HexDigit(AcftType&0x7);
       JSON[Len++]='\"';
       Len+=Format_String(JSON+Len, ",\"acft_cat\":\"");           // GDL90 aircraft category
                           // no-info, para-glider, hang-glider, balloon, glider, powered, heli, UAV
       const uint8_t AcftCat[8] = { 0,          12,          12,      10,      9,       1,    7,  14 } ;
       Len+=Format_Hex(JSON+Len, AcftCat[AcftType&0x07]);
       JSON[Len++]='\"';
       Len+=Format_String(JSON+Len, ",\"no_track\":");
       JSON[Len++]='0' + (AcftType>>3); }
     if(Type==4)                                               // for service/weather
     { uint8_t Service=Msg[0];                                 //
       Len+=Format_String(JSON+Len, ",\"service\":\"");
       if(Service&0x80) JSON[Len++]='I';
       if(Service&0x40) JSON[Len++]='T';
       if(Service&0x20) JSON[Len++]='W';
       if(Service&0x10) JSON[Len++]='H';
       if(Service&0x08) JSON[Len++]='B';
       // if(Service&0x04) JSON[Len++]='R';
       if(Service&0x02) JSON[Len++]='C';
       JSON[Len++]='\"';
       int32_t Lat = getLat(Msg+1);                            // [cordic] decode the latitude
       int32_t Lon = getLon(Msg+4);                            // [cordic] decode the longitude
       Len+=Format_String(JSON+Len, ",\"lat_deg\":");
       Len+=Format_SignDec(JSON+Len, CoordUBX(Lat), 8, 7, 1);
       Len+=Format_String(JSON+Len, ",\"lon_deg\":");
       Len+=Format_SignDec(JSON+Len, CoordUBX(Lon), 8, 7, 1);
       int Idx=7;
       if(Service&0x40)
       { Len+=Format_String(JSON+Len, ",\"temp_deg\":");
         Len+=Format_SignDec(JSON+Len, (int16_t)5*((int8_t)Msg[Idx++]), 2, 1, 1); }
       if(Service&0x20)
       { uint16_t Dir  = Msg[Idx++];                                 // [cordic]
         Len+=Format_String(JSON+Len, ",\"wind_deg\":");
         Len+=Format_UnsDec(JSON+Len, (45*Dir+16)>>5, 2, 1);
         uint16_t Wind = getSpeed(Msg[Idx++]);                       // [0.2km/h]
         Len+=Format_String(JSON+Len, ",\"wind_kmh\":");
         Len+=Format_UnsDec(JSON+Len, 2*Wind, 2, 1);
         uint16_t Gust = getSpeed(Msg[Idx++]);
         Len+=Format_String(JSON+Len, ",\"gust_kmh\":");
         Len+=Format_UnsDec(JSON+Len, 2*Gust, 2, 1); }
       if(Service&0x10)
       { Len+=Format_String(JSON+Len, ",\"hum_perc\":");
         Len+=Format_UnsDec(JSON+Len, (uint16_t)4*Msg[Idx++], 2, 1); }
       if(Service&0x08)
       { Len+=Format_String(JSON+Len, ",\"press_hpa\":");
         Len+=Format_UnsDec(JSON+Len, getPressure(Msg+Idx), 2, 1);
         Idx+=2; }
     }
     if(Type==1 || Type==7)                                    // airborne or ground position
     { int32_t Lat = getLat(Msg);                              // [cordic] decode the latitude
       int32_t Lon = getLon(Msg+3);                            // [cordic] decode the longitude
       Len+=Format_String(JSON+Len, ",\"lat_deg\":");
       Len+=Format_SignDec(JSON+Len, CoordUBX(Lat), 8, 7, 1);
       Len+=Format_String(JSON+Len, ",\"lon_deg\":");
       Len+=Format_SignDec(JSON+Len, CoordUBX(Lon), 8, 7, 1); }
     if(Type==1)                                               // for airpborne position
     { uint32_t Alt=getAltitude(Msg+6);                        // [m] decode the altitude
       uint32_t Speed=getSpeed(Msg[8]);                        // [0.5km/h] ground speed
       Speed = (Speed*355+0x80)>>8;                            // [0.5km/h] => [0.1m/s] convert
       int32_t Climb=getClimb(Msg[9]);                         // [0.1m/s] climb rate
       uint16_t Dir=getDir(Msg[10]);                           // [deg]
       Len+=Format_String(JSON+Len, ",\"alt_msl_m\":");
       Len+=Format_UnsDec(JSON+Len, Alt);
       Len+=Format_String(JSON+Len, ",\"track_deg\":");
       Len+=Format_UnsDec(JSON+Len, Dir);
       Len+=Format_String(JSON+Len, ",\"speed_mps\":");
       Len+=Format_UnsDec(JSON+Len, Speed, 2, 1);
       Len+=Format_String(JSON+Len, ",\"climb_mps\":");
       Len+=Format_SignDec(JSON+Len, Climb, 2, 1, 1);
       if(MsgLen>11)
       { int16_t Turn=getTurnRate(Msg[11]);
         Len+=Format_String(JSON+Len, ",\"turn_dps\":");
         Len+=Format_SignDec(JSON+Len, Turn*10/4, 2, 1, 1); }
       if(MsgLen>12)
       { int32_t AltStd=Alt; Alt+=getQNE(Msg[12]);
         Len+=Format_String(JSON+Len, ",\"alt_std_m\":");
         Len+=Format_SignDec(JSON+Len, AltStd, 1, 0, 1); }
       Len+=Format_String(JSON+Len, ",\"on_ground\":0"); }
     if(Type==7)                                               // for ground position
     { uint8_t Status = Msg[6];
       Len+=Format_String(JSON+Len, ",\"no_track\":1");
       JSON[Len++]='0' + (Status&1);
       Len+=Format_String(JSON+Len, ",\"on_ground\":1"); }
     return Len; }

   int WriteAPRS(char *Out, uint8_t AddrType=0)
   { bool Report=0;
     if(AddrType==0) AddrType = getAddrType();                 // 2 (FLARM) or 3 (OGN)
     int Len=0;
     bool isPosition = Type()==1 || Type()==4 || Type()==7;
     Len+=Format_String(Out+Len, "FNT");
     Len+=Format_Hex(Out+Len, Byte[1]);
     Len+=Format_Hex(Out+Len, Byte[3]);
     Len+=Format_Hex(Out+Len, Byte[2]);
     Len+=Format_String(Out+Len, ">OGNFNT,qOR:");
     Out[Len++]=isPosition?'/':'>';
     Len+=Format_HHMMSS(Out+Len, SlotTime());
     Out[Len++]='h';
     const uint8_t *Msg = this->Msg();
     uint8_t  MsgLen = this->MsgLen();
     switch(Type())
     { case 2:                                 // Name: pilot or weather station
       { Len+=Format_String(Out+Len, " Name=\"");
         for(int Idx=0; Idx<MsgLen; Idx++)
         { uint8_t Byte=Msg[Idx]; if(Byte==0) break;
           Out[Len++]=Byte; }
         Out[Len++]='\"';
         Report=Byte[1]!=0x06; break; }        // do not report names of the Burnair weather stations
       case 4:                                 // Service: mostly meteo
       { uint8_t Service=Msg[0];
         int32_t Lat = getLat(Msg+1);          // [cordic]
         int32_t Lon = getLon(Msg+4);          // [cordic]
         int Idx=7;
         int8_t Temp;                          // [0.5degC]
         if(Service&0x40) Temp=Msg[Idx++];     // if temperature
         uint16_t Dir; uint16_t Speed; uint16_t Gust;
         if(Service&0x20)                      // [if wind data]
         { Dir   = getDir(Msg[Idx++]);         // [deg]
           Speed = getSpeed(Msg[Idx++]);       // [0.2km/h]
           Gust  = getSpeed(Msg[Idx++]); }     // [0.2km/h]
         uint16_t Hum;                         // [0.4%]
         if(Service&0x10) Hum=Msg[Idx++];      // if humidity
         uint16_t Press;                       // [0.1hPa]
         if(Service&0x08) Press=getPressure(Msg+Idx); // if pressure
         Idx+=2;
         char hLat, hLon;
         Len+=Format_Lat(Out+Len, Lat, hLat);
         Out[Len++]='/';
         Len+=Format_Lon(Out+Len, Lon, hLon);
         Out[Len++]='_';
         if(Service&0x20)
         { Len+=Format_UnsDec(Out+Len, Dir, 3);          // [deg]
           Out[Len++]='/';
           Len+=Format_UnsDec(Out+Len, (Speed+4)>>3, 3); // [0.2km/h -> mph]
           Out[Len++]='g';
           Len+=Format_UnsDec(Out+Len, (Gust+4)>>3, 3);  // [0.2km/h -> mph]
         } else Len+=Format_String(Out+Len, ".../...g...");
         Out[Len++]='t';
         if(Service&0x40)
         { // int16_t Fahr=Temp; Fahr+=4*Temp/5; Fahr/=2; Fahr+=32;          //
           int16_t Fahr = (((int16_t)Temp*115+64)>>7) + 32;                 // [0.5degC] => [degF]
           if(Fahr>=0) Len+=Format_UnsDec(Out+Len, Fahr, 3);
                  else Len+=Format_SignDec(Out+Len, Fahr, 2);
         } else Len+=Format_String(Out+Len, "...");
         if(Service&0x10)
         { Out[Len++]='h';
           Hum = (Hum*4+5)/10; if(Hum>=100) Hum=00;
           Len+=Format_UnsDec(Out+Len, Hum, 2); }
         if(Service&0x08)
         { Out[Len++]='b';
           Len+=Format_UnsDec(Out+Len, Press, 5); }
         Report=Byte[1]!=0x06; break; }                            // don't report Burnair weather reports
       case 1:                                                     // airborne position
       { const char *AcftIcon[8] = { "/z", "/g", "/g", "/O", "/'", "\\^", "/X", "/'" } ; // APRS icons for aircraft types
         const uint8_t OGNtype[8] = {   0,    7,    6,  0xB,    1,     8,    3,  0xD } ; // OGN aircraft types
         uint8_t AcftType=Msg[7]>>4;                               // aircraft-type and online-tracking flag
         const char *Icon = AcftIcon[AcftType&7];                  // APRS icon
         uint32_t ID = (OGNtype[AcftType&7]<<2) | AddrType;        // acft-type and addr-type
         bool Track = AcftType&0x08;                               // online tracking flag
         if(!Track) ID|=0x80;                                      // if no online tracking the set as stealth flag
         ID<<=24; ID |= getAddr();                                 // address
         int32_t Lat = getLat(Msg);                                // [cordic]
         int32_t Lon = getLon(Msg+3);                              // [cordic]
         uint32_t Alt=getAltitude(Msg+6);                          // [m]
         uint32_t Feet = ((int32_t)3360*Alt+512)>>10;              // [feet]
         uint32_t Speed=getSpeed(Msg[8]);                          // [0.5km/h]
         uint32_t Knots=(Speed*553+1024)>>11;                      // knots
          int32_t Climb=getClimb(Msg[9]);                          // [0.1m/s]
          int32_t ClimbFeet = ((int32_t)1968*Climb+50)/100;        // [fpm]
         uint16_t Dir=getDir(Msg[10]);                             // [deg]
          int16_t Turn=getQNE(Msg[11]);                            // [0.25deg/s]
          int16_t QNE=getQNE(Msg[12]);                             // [m]
          int32_t StdAlt=Alt+QNE; if(StdAlt<0) StdAlt=0;           // [m]
         uint32_t StdFeet = ((int32_t)3360*StdAlt+512)>>10;        // [feet]
         char hLat, hLon;
         Len+=Format_Lat(Out+Len, Lat, hLat);
         Out[Len++]=Icon[0];
         Len+=Format_Lon(Out+Len, Lon, hLon);
         Out[Len++]=Icon[1];
         Len+=Format_UnsDec(Out+Len, Dir, 3);
         Out[Len++]='/';
         Len+=Format_UnsDec(Out+Len, Knots, 3);
         Len+=Format_String(Out+Len, "/A=");
         Len+=Format_UnsDec(Out+Len, Feet, 6);
         Len+=Format_String(Out+Len, " !W");
         Out[Len++]=hLat;
         Out[Len++]=hLon;
         Out[Len++]='!';
         Len+=Format_String(Out+Len, " id");
         Len+=Format_Hex(Out+Len, ID);
         Out[Len++]=' ';
         Len+=Format_SignDec(Out+Len, ClimbFeet);
         Len+=Format_String(Out+Len, "fpm");
         if(MsgLen>11)
         { Out[Len++]=' ';
           Len+=Format_SignDec(Out+Len, Turn*5/6, 2, 1);
           Len+=Format_String(Out+Len, "rot"); }
         if(MsgLen>12)
         { Len+=Format_String(Out+Len, " FL");
           Len+=Format_UnsDec(Out+Len, StdFeet, 5, 2); }
         Len+=Format_String(Out+Len, " FNT1"); Out[Len++]='0'+(AcftType&7);
         Report=1; break; }
       case 7:                                                     // ground position
       { // const char *StatusMsg[16] = { 0, "Walking", "Vehicle", "Bike", "Boot", 0, 0, 0,
         //                                "Need-ride", "Landed-well", 0, 0, "Need-technical",
         //                                "Need-medical", "Distress(man)", "Distress(auto)" } ;
         uint8_t Status = Msg[6];
         bool Track = Status&1;
         Status>>=4;
         const char *Icon = "\\n";                                 // static object
         if(Status>=13) Icon = "\\!";                              // Emergency
         // const char *StatMsg = StatusMsg[Status];
         uint8_t AcftType = 15;                                    //
         uint32_t ID = (AcftType<<2) | AddrType;                   // acft-type and addr-type
         if(!Track) ID|=0x80;                                      // stealth flag
         ID<<=24; ID |= getAddr();                                 // address
         int32_t Lat = getLat(Msg);                                // [cordic]
         int32_t Lon = getLon(Msg+3);                              // [cordic]
         char hLat, hLon;
         Len+=Format_Lat(Out+Len, Lat, hLat);
         Out[Len++]=Icon[0];
         Len+=Format_Lon(Out+Len, Lon, hLon);
         Out[Len++]=Icon[1];
         Len+=Format_String(Out+Len, " !W");
         Out[Len++]=hLat;
         Out[Len++]=hLon;
         Out[Len++]='!';
         Len+=Format_String(Out+Len, " id");
         Len+=Format_Hex(Out+Len, ID);
         // if(StatMsg)
         // { Out[Len++]=' '; Len+=Format_String(Out+Len, StatMsg); }
         Len+=Format_String(Out+Len, " FNT7"); Out[Len++]=HexDigit(Status);
         Report=1; break; }
     }
     Out[Len++]=' '; Out[Len++]='s';
     Len+=Format_Hex(Out+Len, Sync);
     if(SNR>0)
     { Out[Len++]=' ';
       Len+=Format_SignDec(Out+Len, ((int16_t)SNR*10+2-843)>>2, 2, 1, 1);
       Out[Len++]='d'; Out[Len++]='B'; }
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, FreqOfs/10, 2, 1);
     Len+=Format_String(Out+Len,"kHz");
     if(BitErr)
     { Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, BitErr); Out[Len++]='e'; }
     if(!Report) Len=0;                                             // if not to be reported
     Out[Len]=0; return Len; }

} ;

// =========================================================================================

#ifndef ARDUINO

class FANET_Name
{ public:
   static const int MaxSize = 32;
   uint32_t Time;
   // uint8_t Type;
   char Name[MaxSize];

 public:
  FANET_Name() { Time=0; Name[0]=0; }

  int Copy(const uint8_t *Src, int Size)
  { if(Size>=MaxSize) Size=MaxSize-1;
    memcpy(Name, Src, Size); Name[Size]=0;
    return Size; }

} ;

#include <map>

class FANET_NameList
{ public:
   std::map<uint32_t, FANET_Name> List;

  public:
   int Update(FANET_RxPacket &Packet)
   { if(Packet.Type()!=2) return 0;
     uint32_t Addr = Packet.getAddr();
     FANET_Name &Name = List[Addr];
     Name.Time = Packet.SlotTime();
     Name.Copy(Packet.Msg(), Packet.MsgLen());
     return 1; }

} ;
#endif

// ===============================================================================================

#endif // __FANET_H__
