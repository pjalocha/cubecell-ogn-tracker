#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <algorithm>

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
static uint32_t CRC31_PassBit(uint32_t CRC, uint8_t Bit)     // pass a single bit through the CRC polynomial
{ const uint32_t Poly = 0xC1E52417;
  CRC = (CRC<<1) | Bit;
  if(CRC&0x80000000) CRC ^= Poly;
  return CRC; }

static uint32_t CRC31_PassByte(uint32_t CRC, uint8_t Byte)     // pass a byte through the CRC polynomial
{ for(uint8_t Bit=0; Bit<8; Bit++)
  { CRC = CRC31_PassBit(CRC, Byte>>7);
    Byte<<=1; }
  return CRC; }

const int SignBytes = 64+4;
const int SignBits = SignBytes*8;

static uint32_t SetSignCRC(uint8_t *Sign)
{ uint32_t CRC = 0;
  for(uint8_t Idx=0; Idx<64; Idx++)
  { CRC = CRC31_PassByte(CRC, Sign[Idx]); }
  CRC = CRC31_PassBit(CRC, Sign[64]>>7);
  for(uint8_t Idx=0; Idx<31; Idx++)
  { CRC = CRC31_PassBit(CRC, 0); }
  Sign[64] = (Sign[64]&0x80) | (CRC>>24);
  Sign[65] = CRC>>16;
  Sign[66] = CRC>> 8;
  Sign[67] = CRC    ;
  return CRC; }

static uint32_t CheckSignCRC(const uint8_t *Sign)
{ uint32_t CRC = 0;
  for(uint8_t Idx=0; Idx<68; Idx++)
  { CRC = CRC31_PassByte(CRC, Sign[Idx]); }
  return CRC; }

// ======================================================================================================

static uint32_t CRC31_Syndrome[SignBits];
static uint64_t CRC31_SyndromeSorted[SignBits];

static void SyndromeSort(void)
{ for(int Bit=0; Bit<SignBits; Bit++)
  { uint64_t Syndr = CRC31_Syndrome[Bit]; Syndr = (Syndr<<32) | Bit;
    CRC31_SyndromeSorted[Bit] = Syndr; }
  std::sort(CRC31_SyndromeSorted, CRC31_SyndromeSorted+SignBits); }

uint16_t CRC31_FindSyndrome(uint32_t Syndr)
{ uint16_t Bot=0;
  uint16_t Top=SignBits;
  uint32_t MidSyndr=0;
  for( ; ; )
  { uint16_t Mid=(Bot+Top)>>1;
    MidSyndr = CRC31_SyndromeSorted[Mid]>>32;
    if(Syndr==MidSyndr) return (uint16_t)CRC31_SyndromeSorted[Mid];
    if(Mid==Bot) break;
    if(Syndr< MidSyndr) Top=Mid;
                   else Bot=Mid; }
  return SignBits; }

// ======================================================================================================

static void FlipBit(uint8_t *Data, int Bit)  // flip a single bit of the packet
{ int Idx=Bit>>3; Bit=(Bit&7)^7;
  uint8_t Mask = 1<<Bit;
  Data[Idx]^=Mask; }

static uint32_t BitWeiSorted[SignBits];

static int SignErrCorr(uint8_t *Sign, uint32_t &Syndr, const float *RxSign, float RxAmpl)
{ if(Syndr==0) return 0;
  for(int Bit=0; Bit<SignBits; Bit++)
  { int32_t BitWei = floor(RxSign[Bit]*1024/RxAmpl+0.5);
    BitWei = abs(BitWei); if(BitWei>2048) BitWei=2048;
    BitWeiSorted[Bit] = (BitWei<<16) | Bit; }
  std::sort(BitWeiSorted, BitWeiSorted+SignBits);
  for(int Idx=0; Idx<64; Idx++)
  { uint16_t Bit1 = (uint16_t)BitWeiSorted[Idx];
    uint16_t Wei1 = BitWeiSorted[Idx]>>16;
    if(Wei1>=512) break;
    uint16_t Bit2 = CRC31_FindSyndrome(Syndr^CRC31_Syndrome[Bit1]);
    if(Bit2<SignBits)
    { FlipBit(Sign, Bit1); Syndr^=CRC31_Syndrome[Bit1];
      FlipBit(Sign, Bit2); Syndr^=CRC31_Syndrome[Bit2];
      return 2; }
  }
  for(int Idx1=1; Idx1<64; Idx1++)
  { uint16_t Bit1 = (uint16_t)BitWeiSorted[Idx1];
    uint16_t Wei1  = BitWeiSorted[Idx1]>>16;
    if(Wei1>=512) break;
    for(int Idx2=0; Idx2<Idx1; Idx2++)
    { uint16_t Bit2 = (uint16_t)BitWeiSorted[Idx2];
      uint16_t Wei2  = BitWeiSorted[Idx2]>>16;
      uint16_t Bit3 = CRC31_FindSyndrome(Syndr^CRC31_Syndrome[Bit1]^CRC31_Syndrome[Bit2]);
      if(Bit3<SignBits)
      { FlipBit(Sign, Bit1); Syndr^=CRC31_Syndrome[Bit1];
        FlipBit(Sign, Bit2); Syndr^=CRC31_Syndrome[Bit2];
        FlipBit(Sign, Bit3); Syndr^=CRC31_Syndrome[Bit3];
        return 3; }
    }
  }
  for(int Idx1=2; Idx1<64; Idx1++)
  { uint16_t Bit1 = (uint16_t)BitWeiSorted[Idx1];
    uint16_t Wei1  =          BitWeiSorted[Idx1]>>16;
    if(Wei1>=512) break;
    for(int Idx2=1; Idx2<Idx1; Idx2++)
    { uint16_t Bit2 = (uint16_t)BitWeiSorted[Idx2];
      // uint16_t Wei2  =          BitWeiSorted[Idx2]>>16;
      for(int Idx3=0; Idx3<Idx2; Idx3++)
      { uint16_t Bit3 = (uint16_t)BitWeiSorted[Idx3];
        // uint16_t Wei3  =          BitWeiSorted[Idx3]>>16;
        uint16_t Bit4 = CRC31_FindSyndrome(Syndr^CRC31_Syndrome[Bit1]^CRC31_Syndrome[Bit2]^CRC31_Syndrome[Bit3]);
        if(Bit4<SignBits)
        { FlipBit(Sign, Bit1); Syndr^=CRC31_Syndrome[Bit1];
          FlipBit(Sign, Bit2); Syndr^=CRC31_Syndrome[Bit2];
          FlipBit(Sign, Bit3); Syndr^=CRC31_Syndrome[Bit3];
          FlipBit(Sign, Bit4); Syndr^=CRC31_Syndrome[Bit4];
          return 3; }
      }
    }
  }
  return 0; }

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
static int Count_FixedCRC = 0;
static int Count_RxBitErr = 0;
static int Count_FalseCorr = 0;

static void Count_Clear(void)
{ Count_Trials  = 0;
  Count_GoodCRC = 0;
  Count_BadCRC  = 0;
  Count_FixedCRC = 0;
  Count_RxBitErr = 0;
  Count_FalseCorr = 0; }

static void Count_Print(void)
{ printf("All:%5d GoodCRC:%5d Fixed:%5d False:%3d BitErr:%6d\n",
          Count_Trials, Count_GoodCRC, Count_FixedCRC, Count_FalseCorr, Count_RxBitErr); }

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
  if(Syndr!=0)
  { uint16_t BadBit = CRC31_FindSyndrome(Syndr);
    // printf("Syndr:%08X => [%04X]\n", Syndr, BadBit);
    if(BadBit<SignBits)
    { FlipBit(DecodedSign, BadBit);
      Syndr^=CRC31_Syndrome[BadBit];
      Count_FixedCRC++; }
  }
  if(Syndr!=0)
  { int ErrCorr=SignErrCorr(DecodedSign, Syndr, RxSign, Ampl);
    if(ErrCorr) Count_FixedCRC++; }
  if(Syndr==0) Count_GoodCRC++;
          else Count_BadCRC++;
  float dB = 20.0*log10(Ampl/Noise);
  // printf("TRXsim[%4.1fdB]: %08X %2de\n", dB, Syndr, BitErr);
  return 0; }

// ======================================================================================================

static uint8_t Sign[SignBytes];               // signature to be transmitted, including the CRC

int main(int argc, char *argv[])
{ time_t Now; time(&Now); srand(Now);

  SetRandom(Sign, SignBytes);                   // a random signature
  uint32_t CRC = SetSignCRC(Sign);              // add the CRC on the last 31 bits
  // printf("Sign = "); PrintBytes(Sign, SignBytes); printf(" CRC=%08X\n", CRC);

  // check syndrome for correct frames and with single bit errors
  uint32_t Check = CheckSignCRC(Sign);
  // printf("[----] = %08X\n", Check);
  for(int Bit=0; Bit<SignBits; Bit++)
  { FlipBit(Sign, Bit);
    uint32_t Check = CheckSignCRC(Sign);
    // printf("[%4d] = %08X\n", Bit, Check);
    CRC31_Syndrome[Bit]=Check;                  // store single bit error syndrome in a tabel
    FlipBit(Sign, Bit); }
  SyndromeSort();                               // produce sorted table for a quick search

  // test if syndrome search es correctly
  for(int Bit=0; Bit<SignBits; Bit++)
  { int ErrBit=CRC31_FindSyndrome(CRC31_Syndrome[Bit]);
    if(ErrBit!=Bit) printf("CRC31_FindSyndrome() failed %d<=>%d\n", Bit, ErrBit); }

  float Ampl=1.0;                               // simulate bit errors and packet errors
  float Step=pow(10.0, 0.05);                   // step by 1dB
  for(float Noise=0.1; Noise<0.6; Noise*=Step)  // start with 10:1 amplitude thus 20dB SNR
  { Count_Clear();
    for(int Test=0; Test<10000; Test++)
    { TRXsim(Ampl, Noise); }
    printf("%4.1fdB: ", 20*log10(Ampl/Noise));
    Count_Print(); }

  return 0; }


