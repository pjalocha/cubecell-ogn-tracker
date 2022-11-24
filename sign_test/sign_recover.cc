#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <string.h>

#include <secp256k1.h>             // classical encryption stuff
#include <secp256k1_recovery.h>    // more advanced: recover public key from the signature - library needs to be configured with --enable-module-recovery

static void SetRandom(uint8_t *Data, int Bytes)            // a primitive method to produce random bytes
{ for(int Idx=0; Idx<Bytes; Idx++)
    Data[Idx] = rand();
}

static void PrintBytes(const uint8_t *Data, int Bytes)     // hex-print give n number of bytes
{ for(int Idx=0; Idx<Bytes; Idx++)
    printf("%02X", Data[Idx]);
}

static int ReadBytes(uint8_t *Data, int MaxBytes, const char *Inp)
{ int Len=0;
  for( ; Len<MaxBytes; )
  { int Byte; if(sscanf(Inp, "%02X", &Byte)!=1) break;
    Inp+=2; Data[Len++]=Byte; }
  return Len; }

int main(int argc, char *argv[])
{ time_t Now; time(&Now);
  srand(Now);                      // start standard random number generator from current time, thus each time the random bumbers are different.

  uint8_t Random[32];              // space for random bytes

  secp256k1_pubkey PubKey;         // Public Key: some internal representation, 64 bytes, not portable
  uint8_t ComprPubKey[33];         // compressed, external form: portable, 33 bytes

  uint8_t MsgHash[32];             // Message Hash: 32 bytes

  secp256k1_ecdsa_signature Sig;   // Signature: internal represantation, 64 bytes, not portable
  uint8_t CompSig[64];             // compact form of the signature, portable, 64-bytes

  secp256k1_ecdsa_recoverable_signature RecSig; // signature which is one byte longer which allows for recovery of the Public Key
  uint8_t CompRecSig[68];          // compact form of the signature, portable, 64-bytes
  int     RecID;                   // recovery ID to make possible Public Key recovery

  if(argc<3) return 0;

  // Create a "context" for sign and verify operations
  secp256k1_context* Ctx = secp256k1_context_create(SECP256K1_CONTEXT_SIGN | SECP256K1_CONTEXT_VERIFY); // Contect for sign and verify
  SetRandom(Random, 32);
  int CtxOK = secp256k1_context_randomize(Ctx, Random);              // randomize the context
  printf("CtxOK = %d\n", CtxOK);

  int HashLen = ReadBytes(MsgHash, 32, argv[1]);                     // read the Message Hash
  printf("Hash[%2d] ", HashLen); PrintBytes(MsgHash, HashLen); printf("\n");

  int SignLen = ReadBytes(CompRecSig, 68, argv[2]);                  // read the recoverable signature
  printf("Sign[%2d] ", SignLen); PrintBytes(CompRecSig, SignLen); RecID=CompRecSig[64]>>6; RecID=0; printf(" RecID:%d\n", RecID);

                                                                     // Parse the Compact Recoverable Signature
  int RecSigOK = secp256k1_ecdsa_recoverable_signature_parse_compact(Ctx, &RecSig, CompRecSig, RecID); // parse the Recoverable Signature
  printf("RecSigOK = %d\n", RecSigOK);
                                                                     // convert to "normal" signature
  int ConvOK = secp256k1_ecdsa_recoverable_signature_convert(Ctx, &Sig, &RecSig);
  printf("ConvOK = %d\n", ConvOK);

  if(argc>3)                                                         // if public key is given
  { int KeyLen = ReadBytes(ComprPubKey, 33, argv[3]);
    printf("PubKey[%2d] ", KeyLen); PrintBytes(ComprPubKey, KeyLen); printf("\n");

    size_t PubKeyLen=33;
    int PubKeyOK = secp256k1_ec_pubkey_parse(Ctx, &PubKey, ComprPubKey, PubKeyLen); // parse the Public Key
    printf("PubKeyOK = %d\n", PubKeyOK);

    int VerOK = secp256k1_ecdsa_verify(Ctx, &Sig, MsgHash, &PubKey); // Verify the Signature
    printf("VerOK = %d\n", VerOK);
  }
                                                                     // Recover the Public Key from the Recoverable Signature and the Message Hash
  int RecPubKeyOK = secp256k1_ecdsa_recover(Ctx, &PubKey, &RecSig, MsgHash);
  printf("RecPubKeyOK = %d\n", RecPubKeyOK);

  size_t PubKeyLen=33;
  secp256k1_ec_pubkey_serialize(Ctx, ComprPubKey, &PubKeyLen, &PubKey, SECP256K1_EC_COMPRESSED); // recovered Public Key in compressed form
  printf("RecPubKey = "); PrintBytes(ComprPubKey, PubKeyLen); printf("\n");

  int VerOK = secp256k1_ecdsa_verify(Ctx, &Sig, MsgHash, &PubKey);        // Verify the Signature with the recovered Public Key
  printf("VerOK = %d\n", VerOK);

  secp256k1_context_destroy(Ctx);

}
