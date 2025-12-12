#pragma once

#include "main.h"

#include "ogn.h"
#include "ldpc.h"
#include "fifo.h"
#include "freqplan.h"
#include "lowpass2.h"

#include "rx-pkt.h"

extern FreqPlan Radio_FreqPlan;
extern FIFO<FSK_RxPacket, 16> RxFIFO;
extern Delay<uint8_t, 64> RX_OGN_CountDelay;
extern uint16_t           RX_OGN_Count64;
extern LowPass2<int32_t, 4,2,4> RX_RSSI;

extern    bool RF_Slot;
extern uint8_t RF_Channel;
extern uint8_t RF_SysID;
extern uint8_t RX_OGN_Packets;

extern LDPC_Decoder      Decoder;

void Radio_TxConfig(uint8_t SysID);
void Radio_RxConfig(uint8_t SysID);

// inline void OGN_TxConfig(void) { Radio_TxConfig(Radio_SysID_OGN); }
// inline void OGN_RxConfig(void) { Radio_RxConfig(Radio_SysID_OGN); }
#ifdef WITH_FANET
void FNT_TxConfig(void);
#endif

int OGN_Transmit(const OGN_TxPacket<OGN1_Packet> &TxPacket, uint8_t *Sign=0, uint8_t SignLen=68);
#ifdef WITH_ADSL
int ADSL_Transmit(const ADSL_Packet &TxPacket, uint8_t *Sign=0, uint8_t SignLen=68);
#endif

void Radio_Init(void);

