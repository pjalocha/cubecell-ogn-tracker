#include <uECC.h>

class uECC_SignKey
{ public:
   uint8_t PrivateKey[32];
   uint8_t PublicKey[64];
   uint32_t CRC;

   static const uint32_t FlashPageSize = 256;
   static const uint32_t FlashAddr = 507*FlashPageSize;              // 510-511 are taken, 508-509 are the OGN Parameters

   const struct uECC_Curve_t *Curve;

  public:
   void Init(void)
   { Curve = uECC_secp256k1();
     ReadFromFlash();
     if(CRC!=CalcCRC())
     { uECC_make_key(PublicKey, PrivateKey, Curve);
       WriteToFlash(); }
   }

   int Sign(uint8_t *Sign, const uint8_t *MsgHash, unsigned HashSize=32)    // sign given message hash
   { return uECC_sign(PrivateKey, MsgHash, HashSize, Sign, Curve); } // return 1 for success, 0 for failure

   int ReadFromFlash(void)
   { const uint32_t Bytes = 32+64+4;
     return FLASH_read_at(FlashAddr, (uint8_t *)this, Bytes); }

    int  WriteToFlash(void)
    { const uint32_t Bytes = 32+64+4;
      CRC = CalcCRC();
      return FLASH_update(FlashAddr, (uint8_t *)this, Bytes); }

   uint32_t CalcCRC(void)
   { uint32_t CRC=0xFFFFFFFF;
     CRC = crc32(CRC, PrivateKey, 32);
     CRC = crc32(CRC, PublicKey , 64);
     return ~CRC; }

   uint32_t crc32(uint32_t CRC, const uint8_t *Data, int Bytes)
   { for(int Idx=0; Idx<Bytes; Idx++)
     { uint8_t Byte=Data[Idx];
       for(int Bit=0; Bit<8; Bit++)
       { uint32_t X = (Byte^CRC)&1;
         CRC>>=1;
         if(X) CRC^=0xEDB88320;
         Byte>>=1; }
     }
     return ~CRC; }

} ;

