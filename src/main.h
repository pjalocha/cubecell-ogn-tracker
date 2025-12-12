
#pragma once

#include "Arduino.h"

#include <stdint.h>

#define WITH_OGN1                          // OGN protocol version 1
#define OGN_Packet OGN1_Packet

typedef union
{ uint64_t Word;
  struct
  { uint32_t RX;
    uint32_t GPS;
  } ;
} Random64;

extern Random64 Random;

#define DEFAULT_AcftType        1         // [0..15] default aircraft-type: glider
#define DEFAULT_GeoidSepar     40         // [m]
#define DEFAULT_CONbaud    115200         // [bps]
#define DEFAULT_PPSdelay       50         // [ms]
#define DEFAULT_FreqPlan        0

uint64_t getUniqueID(void);
uint32_t getUniqueAddress(void);

#define HARDWARE_ID 0x03
#define SOFTWARE_ID 0x01

#define HARD_NAME "OGN-CC"
// #define SOFT_NAME "2023.05.28"
#define SOFT_NAME "v0.1.8"

#include "parameters.h"

extern FlashParameters Parameters;

extern uint32_t GPS_PPS_ms;
extern uint32_t GPS_PPS_UTC;
