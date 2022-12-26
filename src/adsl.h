#ifndef __ADSL_H__
#define __ADSL_H__

// #include <stdlib.h>
// #include <string.h>
// #include "radiodemod.h"
// #include "intmath.h"
#include "ognconv.h"
// #include "bitcount.h"
// #include "format.h"
// #include "crc1021.h"

class ADSL_Packet
{ public:

   const static uint8_t TxBytes = 27; // including SYNC, Length and 24-bit CRC
   const static uint8_t SYNC1 = 0x72;
   const static uint8_t SYNC2 = 0x4B;

   uint8_t SYNC[2];          // two bytes for correct alignment: can contain the last two SYNC bytes
   uint8_t Length;           // [bytes] packet length = 24 = 0x18 (excluding length but including the 24-bit CRC)
   uint8_t Version;          // Version[4]/Sigmature[1]/Key[2]/Reserved[1]
   union
   { uint32_t Word[5];       // this part to be scrambled/encrypted, is aligned to 32-bit
     struct                  // this is aligned to 32-bit
     { uint8_t Type;         // 2=iConspicuity, bit #7 = Unicast
       uint8_t Address  [4]; // Address[30]/Reserved[1]/RelayForward[1] (not aligned to 32-bit !)
       union
       { uint8_t Meta     [2]; // Time[6]/Cat[5]/Emerg[3]/FlightState[2]
         struct
         { uint8_t TimeStamp   :6; // [0.25sec]
           uint8_t FlightState :2; // 0=unknown, 1=ground, 2=airborne
           uint8_t AcftCat     :5; // 1=light, 2=small-heavy, 3=heli, 4=glider, 5=baloon/airship, 6=para/hang-glider, 7=skydiver, 
           uint8_t Emergency   :3; // 1=OK
         } ;
       } ;
       uint8_t Position[11]; // Lat[24]/Lon[24]/Speed[8]/Alt[14]/Climb[9]/Track[9]
       union
       { uint8_t Integrity[2]; // SourceInteg[2]/DesignAssurance[2]/NavigationIntegrity[4]/NorizAccuracy[3]/VertAccuracy[2]/ValocityAccuracy[2]/Reserved[1]
         struct
         { uint8_t SourceIntegrity:2; // 3=1e-7/h, 2=1e-5/h, 1=1e-3/h
           uint8_t DesignAssurance:2; // 3=B, 2=C, 1=D
           uint8_t NavigIntegrity :4; // 12=7.5m, 11=25m, 10=75m
           uint8_t HorizAccuracy  :3; // 7=3m, 6=10m, 5=30m
           uint8_t VertAccuracy   :2; // 3=15m, 2=45m, 1=150m
           uint8_t VelAccuracy    :2; // 3=1m/s 2=3m/s 3=10m/s
           uint8_t Reserved       :1; //
         } ;
       } ;
     } ;
   } ;
   uint8_t CRC[3];           // 24-bit (is aligned to 32-bit)

   // uint8_t Spare;
   // uint32_t Time;            // [sec] receive time or other estimate

  public:
   void Init(void) { SYNC[0]=SYNC1; SYNC[1]=SYNC2; Length=TxBytes-3; Version=0x00; Type=0x02; }

   void Print(void)
   { printf("ADSL v%02X %3.1fs: %02X:%06X [%+09.5f,%+010.5f]deg %dm %+4.1fm/s %3.1fm/s %05.1fdeg\n",
         Version, 0.25*TimeStamp, getAddrTable(), getAddress(), FNTtoFloat(getLat()), FNTtoFloat(getLon()),
         getAlt(), 0.125*getClimb(), 0.25*getSpeed(), (45.0/0x40)*getTrack());
   }

/*
   uint32_t getAddress(void)   const { return get3bytes(Address); }
   uint8_t  getAddrTable(void) const { return Address[3]&0x3F; }

    void    setAddress(uint32_t Addr)   { set3bytes(Address, Addr); }
    void    setAddrTable(uint8_t Table) { Address[3] = (Address[3]&0xC0) | Table; }
*/

   uint8_t  getRelay(void)     const { return Address[3]&0x80; }
   void     setRelay(uint8_t Relay)  { Address[3] = (Address[3]&0x7F) | (Relay<<7); }

   static uint32_t get3bytes(const uint8_t *Byte) { int32_t Word=Byte[2]; Word<<=8; Word|=Byte[1]; Word<<=8; Word|=Byte[0]; return Word; }
   static void     set3bytes(uint8_t *Byte, uint32_t Word) { Byte[0]=Word; Byte[1]=Word>>8; Byte[2]=Word>>16; }

   static uint32_t get4bytes(const uint8_t *Byte)
   { uint32_t Word =Byte[3]; Word<<=8;
              Word|=Byte[2]; Word<<=8;
              Word|=Byte[1]; Word<<=8;
              Word|=Byte[0];
     return Word; }
   static void     set4bytes(uint8_t *Byte, uint32_t Word) { Byte[0]=Word; Byte[1]=Word>>8; Byte[2]=Word>>16; Byte[3]=Word>>24; }

   uint32_t getAddress(void) const
   { uint32_t Addr = get4bytes(Address); return (Addr>>6)&0x00FFFFFF; }

   void setAddress(uint32_t NewAddr)
   { uint32_t Addr = get4bytes(Address);
     Addr = (Addr&0xC000003F) | (NewAddr<<6);
     set4bytes(Address, Addr); }

   uint8_t  getAddrTable(void) const { return Address[0]&0x3F; }
    void    setAddrTable(uint8_t Table) { Address[0] = (Address[0]&0xC0) | Table; }

   uint8_t getAddrType(void) const
   { uint8_t Table=getAddrTable();
     if(Table==0x05) return 1;         // ICAO
     if(Table==0x06) return 2;         // FLARM
     if(Table==0x07) return 3;         // OGN
     if(Table==0x08) return 2;         // FANET => FLARM ?
     return 0; }

   void setAddrType(uint8_t AddrType)
   { if(AddrType==0) setAddrTable(0);
     else setAddrTable(AddrType+4); }

   void setAcftType(uint8_t AcftType)
   { const uint8_t Map[16] = { 0, 4, 1, 3,                     // unknown, glider, tow-plane, helicopter
                               8, 1, 7, 7,                     // sky-diver, drop plane, hang-glider, para-glider
                               1, 2, 0, 5,                     // motor airplane, jet, UFO, balloon
                               5,11, 0, 0 } ;                  // airship, UAV, ground vehicle, static object
     if(AcftType<16) AcftCat=Map[AcftType];
                else AcftCat=0; }
   uint8_t getAcftType(void) const
   { const uint8_t Map[32] = { 0, 8, 9, 3, 1,12, 2, 7,
                               4,13, 3,13,13,13, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0 } ;
     return Map[AcftCat]; }

   uint8_t getHorPrec(void) const
   { const uint8_t Map[8] = { 63, 63, 63, 63, 63, 30, 10, 3 } ;
     return Map[HorizAccuracy]; }
   void setHorPrec(uint8_t Prec)
   {      if(Prec<= 3) HorizAccuracy=7;
     else if(Prec<=10) HorizAccuracy=6;
     else if(Prec<=30) HorizAccuracy=5;
     else HorizAccuracy=4;
     VelAccuracy = HorizAccuracy-4; }

   uint8_t getVerPrec(void) const
   { const uint8_t Map[8] = { 63, 63, 45, 15 } ;
     return Map[VertAccuracy]; }
   void setVerPrec(uint8_t Prec)
   {      if(Prec<=15) HorizAccuracy=3;
     else if(Prec<=45) HorizAccuracy=2;
     else HorizAccuracy=1; }

   static int32_t FNTtoUBX(int32_t Coord) { return ((int64_t)900007296*Coord+0x20000000)>>30; } // [FANET-cordic ] => [1e-7 deg]
   static int32_t OGNtoFNT(int32_t Coord) { return ((int64_t)Coord*83399317+(1<<21))>>22; }     // [0.0001/60 deg] => [FANET cordic]
   static int32_t UBXtoFNT(int32_t Coord) { return ((int64_t)Coord*5003959 +(1<<21))>>22; }     // [1e-7 deg]      => [FANET cordic]
   static float   FNTtoFloat(int32_t Coord)                             // convert from FANET cordic units to float degrees
   { const float Conv = 90.0007295677/0x40000000;                       // FANET cordic conversion factor (not exactly cordic)
     return Conv*Coord; }

    int32_t getLat(void) const { int32_t Lat=get3bytes(Position  ); Lat<<=8; Lat>>=1; return Lat; } // FANET-cordic
    int32_t getLon(void) const { int32_t Lon=get3bytes(Position+3); Lon<<=8; return Lon; }          // FANET-cordic

    void    setLat(int32_t Lat)  { Lat = (Lat+0x40)>>7; set3bytes(Position  , Lat); }           // FANET-cordic
    void    setLon(int32_t Lon)  { Lon = (Lon+0x80)>>8; set3bytes(Position+3, Lon); }           // FANET-cordic

    uint16_t getSpeed(void) const { return UnsVRdecode<uint16_t,6>(Position[6]); }              // [0.25 m/s]
    void setSpeed(uint16_t Speed) { Position[6] = UnsVRencode<uint16_t,6>(Speed); }             // [0.25 m/s]

   int32_t getAlt(void) const                                                                  // [m]
   { int32_t Word=Position[8]&0x3F; Word<<=8; Word|=Position[7];
     return UnsVRdecode<int32_t,12>(Word)-316; }
   void setAlt(int32_t Alt)
   { Alt+=316; if(Alt<0) Alt=0;
     int32_t Word=UnsVRencode<uint32_t,12>(Alt);
     Position[7]=Word;
     Position[8] = (Position[8]&0xC0) | (Word>>8); }

   int16_t getClimbWord(void) const                                                             //
   { int16_t Word=Position[9]&0x7F; Word<<=2; Word|=Position[8]>>6; return Word; }
   int16_t getClimb(void) const                                                                 // [0.125 m/s]
   { return SignVRdecode<int16_t,6>(getClimbWord()); }
   void setClimb(int16_t Climb)                                                                 // [0.125 m/s]
   { setClimbWord(SignVRencode<int16_t,6>(Climb)); }
   void setClimbWord(int16_t Word)
   { Position[8] = (Position[8]&0x3F) | ((Word&0x03)<<6);
     Position[9] = (Position[9]&0x80) |  (Word>>2); }
   bool hasClimb(void) { return getClimbWord()!=0x100; }                                        // climb-rate present or absent
   void clrClimb(void) { setClimbWord(0x100); }                                                 // declare climb-rate as absent

   uint16_t getTrack(void) const                                                                // 9-bit cordic
   { int16_t Word=Position[10]; Word<<=1; Word|=Position[9]>>7; return Word; }
   void setTrack(int16_t Word)
   { Position[9] = (Position[9]&0x7F) | ((Word&0x01)<<7);
     Position[10] = Word>>1; }

   void Scramble(void)
   { XXTEA_Encrypt_Key0(Word, 5, 6); }

   void Descramble(void)
   { XXTEA_Decrypt_Key0(Word, 5, 6); }

   static uint32_t PolyPass(uint32_t CRC, uint8_t Byte)     // pass a single byte through the CRC polynomial
   { const uint32_t Poly = 0xFFFA0480;
     CRC |= Byte;
     for(uint8_t Bit=0; Bit<8; Bit++)
     { if(CRC&0x80000000) CRC ^= Poly;
       CRC<<=1; }
     return CRC; }

   static uint32_t checkPI(const uint8_t *Byte, uint8_t Bytes) // run over data bytes and the three CRC bytes
   { uint32_t CRC = 0;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { CRC = PolyPass(CRC, Byte[Idx]); }
     return CRC>>8; }                                          // should be all zero for a correct packet

   static uint32_t calcPI(const uint8_t *Byte, uint8_t Bytes)  // calculate PI for the given packet data excluding the three CRC bytes
   { uint32_t CRC = 0;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { CRC = PolyPass(CRC, Byte[Idx]); }
     CRC=PolyPass(CRC, 0); CRC=PolyPass(CRC, 0); CRC=PolyPass(CRC, 0);
     return CRC>>8; }                                          //

    void setCRC(void)
    { uint32_t Word = calcPI((const uint8_t *)&Version, TxBytes-6);
      CRC[0]=Word>>16; CRC[1]=Word>>8; CRC[2]=Word; }

    uint32_t checkCRC(void) const
    { return checkPI((const uint8_t *)&Version, TxBytes-3); }

} __attribute__((packed));

#endif // __ADSL_H__
