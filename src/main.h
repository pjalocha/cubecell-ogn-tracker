
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
#define SOFT_NAME "HDR-25.01"

#include "parameters.h"

extern FlashParameters Parameters;

extern uint32_t GPS_PPS_ms;
extern uint32_t GPS_PPS_UTC;

uint8_t I2C_Read (uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10);
uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10);

template <class Type>
 uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, Type &Object, uint8_t Wait=10)
{ return I2C_Write(Bus, Addr, Reg, (uint8_t *)&Object, sizeof(Type), Wait); }
template <class Type>
 uint8_t I2C_Read (uint8_t Bus, uint8_t Addr, uint8_t Reg, Type &Object, uint8_t Wait=10)
{ return I2C_Read (Bus, Addr, Reg, (uint8_t *)&Object, sizeof(Type), Wait); }
