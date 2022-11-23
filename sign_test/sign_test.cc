// Demo code for the principle of transmitting digital signature so that the public key can be recovered on the receiving end
// you need to install the secp256k1 library: https://github.com/bitcoin-core/secp256k1.git
// and configure it with the --enable-module-recovery option

// Some details on the principle of the public key recovery and a python example are described here:
// https://medium.com/asecuritysite-when-bob-met-alice/crypto-magic-recovering-alices-public-key-from-an-ecdsa-signature-e7193df8df6e

// for the OGN-Tracker we use the uECC library, which was the only one I manage to compile, link and run on CubeCell
// https://github.com/kmackay/micro-ecc.git

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <string.h>

#include <secp256k1.h>             // classical encryption stuff
#include <secp256k1_recovery.h>    // more advanced: recover public key from the signature - library needs to be configured with --enable-module-recovery

// #include "uecc-signkey.h"

static void SetRandom(uint8_t *Data, int Bytes)            // a primitive method to produce random bytes
{ for(int Idx=0; Idx<Bytes; Idx++)
    Data[Idx] = rand();
}

static void PrintBytes(const uint8_t *Data, int Bytes)     // hex-print give n number of bytes
{ for(int Idx=0; Idx<Bytes; Idx++)
    printf("%02X", Data[Idx]);
}

int main(int argc, char *argv[])
{ time_t Now; time(&Now);
  srand(Now);                      // start standard random number generator from current time, thus each time the random bumbers are different.

  uint8_t Random[32];              // space for random bytes

  uint8_t SecKey[32];              // Private Key: 32 bytes

  secp256k1_pubkey PubKey;         // Public Key: some internal representation, 64 bytes, not portable
  uint8_t ComprPubKey[33];         // compressed, external form: portable, 33 bytes

  uint8_t MsgHash[32];             // Message Hash: 32 bytes

  secp256k1_ecdsa_signature Sig;   // Signature: internal represantation, 64 bytes, not portable
  uint8_t CompSig[64];             // compact form of the signature, portable, 64-bytes

  secp256k1_ecdsa_recoverable_signature RecSig; // signature which is one byte longer which allows for recovery of the Public Key
  uint8_t CompRecSig[64];          // compact form of the signature, portable, 64-bytes
  int     RecID;                   // recovery ID to make possible Public Key recovery

  // Create a "context" for sign and verify operations
  secp256k1_context* Ctx = secp256k1_context_create(SECP256K1_CONTEXT_SIGN | SECP256K1_CONTEXT_VERIFY); // Contect for sign and verify
  SetRandom(Random, 32);
  int CtxOK = secp256k1_context_randomize(Ctx, Random);              // randomize the context
  printf("CtxOK = %d\n", CtxOK);

  // Create a Secret Key
  while(1)
  { SetRandom(SecKey, 32);                                           // random secret key
    if(secp256k1_ec_seckey_verify(Ctx, SecKey)) break; }             // good enough ?
  printf("SecKey = "); PrintBytes(SecKey, 32); printf("\n");

  // Create a Public Key
  int PubCreateOK = secp256k1_ec_pubkey_create(Ctx, &PubKey, SecKey); // create Public Key matched to the Secret Key
  size_t PubKeyLen=33;                                                //
  secp256k1_ec_pubkey_serialize(Ctx, ComprPubKey, &PubKeyLen, &PubKey, SECP256K1_EC_COMPRESSED); // Public Key in compressed form
  printf("PubKey = "); PrintBytes(ComprPubKey, PubKeyLen); printf(" (%d)\n", PubCreateOK); // (hex-printable)

  // Imagine we have a message to sign and we made its hash
  SetRandom(MsgHash, 32);                                            // a random Message Hash
  printf("MsgHsh = "); PrintBytes(MsgHash, 32); printf("\n");        // (hex-printable)

  // Sign the Message Hash
  secp256k1_ecdsa_sign(Ctx, &Sig, MsgHash, SecKey, NULL, NULL);      // Sign the Message Hash - classical

  // Produce Compact Signature, 64-byte long
  secp256k1_ecdsa_signature_serialize_compact(Ctx, CompSig, &Sig);   // Compact Signature
  printf("MsgSig    = "); PrintBytes(CompSig, 64); printf("\n");        // (hex-printable)

  secp256k1_ecdsa_sign_recoverable(Ctx, &RecSig, MsgHash, SecKey, NULL, NULL);   // Sign and produce recoverable signature

  // Produce Compact Signature and Recovery ID, which can take values 0..3 but only 0/1 was observed in this test
  secp256k1_ecdsa_recoverable_signature_serialize_compact(Ctx, CompRecSig, &RecID, &RecSig);   // Compact Recoverable Signature
  printf("MsgRecSig = "); PrintBytes(CompRecSig, 64); printf(" RecID:%02X\n", RecID);        // (hex-printable)

  // on the receiving end...

  // Parse the Compact Signature
  int SigOK = secp256k1_ecdsa_signature_parse_compact(Ctx, &Sig, CompSig); // parse the Signature
  printf("SigOK = %d\n", SigOK);

  // Parse the Compact Recoverable Signature
  int RecSigOK = secp256k1_ecdsa_recoverable_signature_parse_compact(Ctx, &RecSig, CompSig, RecID); // parse the Recoverable Signature
  printf("RecSigOK = %d\n", RecSigOK);

  // Parse the Public Key
  int PubKeyOK = secp256k1_ec_pubkey_parse(Ctx, &PubKey, ComprPubKey, PubKeyLen); // parse the Public Key
  printf("PubKeyOK = %d\n", PubKeyOK);

  // Verify the Signature
  int VerOK = secp256k1_ecdsa_verify(Ctx, &Sig, MsgHash, &PubKey);
  printf("VerOK = %d\n", VerOK);

  // Recover the Public Key from the Signature
  int RecPubKeyOK = secp256k1_ecdsa_recover(Ctx, &PubKey, &RecSig, MsgHash);
  printf("RecPubKeyOK = %d\n", RecPubKeyOK);

  secp256k1_ec_pubkey_serialize(Ctx, ComprPubKey, &PubKeyLen, &PubKey, SECP256K1_EC_COMPRESSED); // recovered Public Key in compressed form
  printf("RecPubKey = "); PrintBytes(ComprPubKey, PubKeyLen); printf("\n"); // (hex-printable)

  secp256k1_context_destroy(Ctx);

  return 0; }

