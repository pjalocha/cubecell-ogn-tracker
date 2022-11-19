#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "bitcount.h"

// ======================================================================================================

static double UniformNoise(void)
{ return ((double)rand()+1.0)/((double)RAND_MAX+1.0); }

static void WhiteNoise(double &I, double &Q)
{ double Power,Phase;
  Power=sqrt(-2*log(UniformNoise()));
  Phase=2*M_PI*UniformNoise();
  I=Power*cos(Phase);
  Q=Power*sin(Phase); }

template <class Float>
 void AddWhiteNoise(Float *Signal, int Size, double Sigma=1.0)
{ double I,Q; int Idx;
  for(Idx=0; Idx<(Size-1); Idx+=2)
  { WhiteNoise(I, Q);
    Signal[Idx]+=Sigma*I;
    Signal[Idx+1]+=Sigma*Q; }
  if(Idx<Size)
  { WhiteNoise(I, Q);
    Signal[Idx]+=Sigma*I; }
}

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
  Sign[64] = (Sign[64]&0x80) | (CRC>>24);
  Sign[65] = CRC>>16;
  Sign[66] = CRC>> 8;
  Sign[67] = CRC    ;
  return CRC; }

static uint32_t CheckSignCRC(const uint8_t *Sign)
{ uint32_t CRC = 0;
  for(uint8_t Idx=0; Idx<68; Idx++)
  { CRC = Poly31_PassByte(CRC, Sign[Idx]); }
  return CRC; }

// ======================================================================================================

static void Encode(float *Tx, const uint8_t *Sign, float Ampl=1.0)
{ for(int Idx=0; Idx<SignBytes; Idx++)
  { uint8_t Byte=Sign[Idx];
    for(int Bit=0; Bit<8; Bit++)
    { float Sig = Byte&0x80 ? Ampl:-Ampl;
      *Tx++ = Sig;
      Byte<<=1; }
  }
}

static void Decode(uint8_t *Sign, const float *Rx)
{ for(int Idx=0; Idx<SignBytes; Idx++)
  { uint8_t Byte=0;
    for(int Bit=0; Bit<8; Bit++)
    { float Sig = *Rx++;
      Byte<<=1;
      Byte |= Sig>0; }
    Sign[Idx] = Byte;
  }
}

static int BitErrors(const uint8_t *Tx, const uint8_t *Rx)
{ int Count=0;
  for(int Idx=0; Idx<SignBytes; Idx++)
  { uint8_t Byte = Tx[Idx] ^ Rx[Idx];
    Count += Count1s(Byte); }
  return Count; }

static uint8_t TxSign[SignBytes];
static float   RxSign[SignBits];
static uint8_t DecodedSign[SignBytes];

static int Count_Trials  = 0;
static int Count_GoodCRC = 0;
static int Count_BadCRC  = 0;
static int Count_RxBitErr = 0;
static int Count_FalseCorr = 0;

static void Count_Clear(void)
{ Count_Trials  = 0;
  Count_GoodCRC = 0;
  Count_BadCRC  = 0;
  Count_RxBitErr = 0;
  Count_FalseCorr = 0; }

static void Count_Print(void)
{ printf("All:%5d GoodCRC:%5d False:%3d BitErr:%6d\n",
          Count_Trials, Count_GoodCRC, Count_FalseCorr, Count_RxBitErr); }

static int TRXsim(float Ampl=1.0, float Noise=0.5)
{ Count_Trials++;
  SetRandom(TxSign, SignBytes);
  SetSignCRC(TxSign);
  Encode(RxSign, TxSign, Ampl);
  AddWhiteNoise(RxSign, SignBits, Noise);
  Decode(DecodedSign, RxSign);
  int BitErr = BitErrors(TxSign, DecodedSign);
  Count_RxBitErr += BitErr;
  uint32_t Syndr = CheckSignCRC(DecodedSign);
  if(Syndr==0) Count_GoodCRC++;
          else Count_BadCRC++;
  float dB = 20.0*log10(Ampl/Noise);
  // printf("TRXsim[%4.1fdB]: %08X %2de\n", dB, Syndr, BitErr);
  return 0; }

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
    // printf("[%3d] = %08X\n", Bit, Check);
    Poly31_Syndrome[Bit]=Check;
    FlipBit(Sign, Bit); }

  float Ampl=1.0;
  for(float Noise=0.1; Noise<0.5; Noise*=1.1)
  { Count_Clear();
    for(int Test=0; Test<10000; Test++)
    { TRXsim(Ampl, Noise); }
    printf("%4.1fdB: ", 20*log10(Ampl/Noise));
    Count_Print(); }

  return 0; }


