#include <stdint.h>

#include <Crypto.h>

#include "uECC.h"

class uECC_SignKey
{ public:
   uint8_t PrivateKey[32]; // Private Key
   uint8_t PublicKey[64];  // Public Key
   uint32_t CRC;           // Check-sum for the data in the flash

   uint8_t MsgHash[32];    // Message Hash
   uint8_t Signature[68];  // Recoverable signature

   static const uint32_t FlashPageSize = 256;
   static const uint32_t FlashAddr = 507*FlashPageSize;              // 510-511 are taken, 508-509 are the OGN Parameters

   SHA256 HashProc;        // SHA-256 processor

   const struct uECC_Curve_t *Curve;

  public:
   void Init(void)
   { Curve = uECC_secp256k1();                          // choose the Curve
     ReadFromFlash();                                   // read from Flash
     if(CRC!=CalcCRC())                                 // if CRC is bad
     { if(uECC_make_key(PublicKey, PrivateKey, Curve))  // produce the pirvte and public key
         WriteToFlash(); }                              // store in Flash
   }

   int CompressPubKey(uint8_t *ComprPubKey)
   { uECC_compress(PublicKey, ComprPubKey, Curve); return 33; }

   int Sign(uint8_t *Sign, const uint8_t *MsgHash, unsigned HashSize=32)    // sign given message hash
   { return uECC_sign(PrivateKey, MsgHash, HashSize, Sign, Curve); }        // return 1 for success, 0 for failure

   void Hash(uint32_t Time, const uint8_t *Packet, uint8_t PktBytes=20)      // Hash an (OGN) packet with the UTC time
   { HashProc.doUpdate((const uint8_t *)&Time, sizeof(uint32_t));
     HashProc.doUpdate(Packet, PktBytes);
     HashProc.doFinal(MsgHash); }

//     int OK=Sign(Signature, MsgHash, 32);
//     // Signature[64] = ???;
//     return OK; }                                                           // 1=success, 0=failure

   static void PrintBytes(const uint8_t *Data, int Bytes)     // hex-print give n number of bytes
   { for(int Idx=0; Idx<Bytes; Idx++)
      printf("%02X", Data[Idx]);
   }

   void PrintKeys(void)
   { printf("PriKey: "); PrintBytes(PrivateKey, 32); printf("\n");
     // printf("PubKey: "); PrintBytes(PublicKey , 64); printf("\n");
     CompressPubKey(Signature);
     printf("PubKey: "); PrintBytes(Signature, 33); printf("\n"); }

   int ReadFromFlash(void)
   { const uint32_t Bytes = 32+64+4;
     return FLASH_read_at(FlashAddr, (uint8_t *)this, Bytes); }

   int  WriteToFlash(void)
   { const uint32_t Bytes = 32+64+4;
     CRC = CalcCRC();
     return FLASH_update(FlashAddr, (uint8_t *)this, Bytes); }

   uint32_t CalcCRC(void)
   { uint32_t CRC=0xFFFFFFFF;
     CRC = CRC32(CRC, PrivateKey, 32);
     CRC = CRC32(CRC, PublicKey , 64);
     return ~CRC; }

   static uint32_t CRC32(uint32_t CRC, const uint8_t *Data, int Bytes)
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

