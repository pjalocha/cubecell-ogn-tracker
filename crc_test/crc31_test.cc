#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

// ======================================================================================================

static void SetRandom(uint8_t *Data, int Bytes)            // a primitive method to produce random bytes
{ for(int Idx=0; Idx<Bytes; Idx++)
    Data[Idx] = rand();
}

static void PrintBytes(const uint8_t *Data, int Bytes)     // hex-print give n number of bytes
{ for(int Idx=0; Idx<Bytes; Idx++)
    printf("%02X", Data[Idx]);
}

// ======================================================================================================

// polynomials from: https://users.ece.cmu.edu/~koopman/crc/crc31.html
static uint32_t Poly31_PassBit(uint32_t CRC, uint8_t Bit)     // pass a single bit through the CRC polynomial
{ const uint32_t Poly = 0xC1E52417;
  CRC = (CRC<<1) | Bit;
  if(CRC&0x80000000) CRC ^= Poly;
  return CRC; }

static uint32_t Poly31_PassByte(uint32_t CRC, uint8_t Byte)     // pass a byte through the CRC polynomial
{ for(uint8_t Bit=0; Bit<8; Bit++)
  { CRC = Poly31_PassBit(CRC, Byte>>7);
    Byte<<=1; }
  return CRC; }

const int SignBytes = 64+4;
const int SignBits = SignBytes*8;

uint32_t Poly31_Syndrome[SignBits];

static uint32_t SetSignCRC(uint8_t *Sign)
{ uint32_t CRC = 0;
  for(uint8_t Idx=0; Idx<64; Idx++)
  { CRC = Poly31_PassByte(CRC, Sign[Idx]); }
  CRC = Poly31_PassBit(CRC, Sign[64]>>7);
  for(uint8_t Idx=0; Idx<31; Idx++)
  { CRC = Poly31_PassBit(CRC, 0); }
  // CRC>>=1; //printf("CRC = %08X\n", CRC);
  Sign[64] = (Sign[64]&0x80) | (CRC>>24);
  Sign[65] = CRC>>16;
  Sign[66] = CRC>> 8;
  Sign[67] = CRC    ;
  return CRC; }
/*
static uint32_t CheckSignCRC(const uint8_t *Sign)
{ uint32_t CRC=0x00000000;
  for(uint8_t Idx=0; Idx<64; Idx++)
  { CRC = Poly31_PassByte(CRC, Sign[Idx]); }
  CRC = Poly31_PassBit(CRC, Sign[64]>>7);
  // CRC>>=1; // printf("CRC = %08X\n", CRC);
  uint32_t Match=Sign[64]&0x7F;
  for(uint8_t Idx=1; Idx<4; Idx++)
  { Match = (Match<<8) | Sign[64+Idx]; }
  return CRC^Match; }
*/
static uint32_t CheckSignCRC(const uint8_t *Sign)
{ uint32_t CRC = 0;
  for(uint8_t Idx=0; Idx<68; Idx++)
  { CRC = Poly31_PassByte(CRC, Sign[Idx]); }
  return CRC; }

// ======================================================================================================


// ======================================================================================================

static void FlipBit(uint8_t *Data, int Bit)  // flip a single bit of the packet
{ int Idx=Bit>>3; Bit=(Bit&7)^7;
  uint8_t Mask = 1<<Bit;
  Data[Idx]^=Mask; }

static uint8_t Sign[SignBytes];               // signature to be transmitted, including the CRC

int main(int argc, char *argv[])
{ time_t Now; time(&Now); srand(Now);

  SetRandom(Sign, SignBytes);
  uint32_t CRC = SetSignCRC(Sign);
  // printf("Sign = "); PrintBytes(Sign, SignBytes); printf(" CRC=%08X\n", CRC);

  uint32_t Check = CheckSignCRC(Sign);
  printf("[---] = %08X\n", Check);
  for(int Bit=0; Bit<SignBits; Bit++)
  { FlipBit(Sign, Bit);
    uint32_t Check = CheckSignCRC(Sign);
    printf("[%3d] = %08X\n", Bit, Check);
    Poly31_Syndrome[Bit]=Check;
    FlipBit(Sign, Bit); }

  return 0; }


