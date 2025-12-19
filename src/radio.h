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

extern    bool Radio_Slot;
extern uint8_t Radio_Channel;
extern uint8_t Radio_SysID;

extern  int8_t Radio_CAD;

extern uint8_t RX_OGN_Packets;

bool Radio_isIdle   (void);
bool Radio_TxRunning(void);
bool Radio_RxRunning(void);

extern LDPC_Decoder      Decoder;

void Radio_TxConfig(uint8_t SysID);
void Radio_RxConfig(uint8_t SysID);

#ifdef WITH_MESHT
void MSH_TxConfig(void);
#endif
#ifdef WITH_FANET
void FNT_TxConfig(void);
#endif

int OGN_ManchTx(const OGN_TxPacket<OGN1_Packet> &TxPacket, uint8_t *Sign=0, uint8_t SignLen=68);
#ifdef WITH_ADSL
int ADSL_ManchTx(const ADSL_Packet &TxPacket, uint8_t *Sign=0, uint8_t SignLen=68);
#endif

void Radio_Init(void);

