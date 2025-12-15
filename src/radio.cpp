
#include "main.h"

#include "radio.h"

#include "LoRaWan_APP.h"
#include "sx126x.h"

extern "C" {
#include "sx126x-board.h"
}

#include "manchester.h"
#include "paw.h"

bool    RF_Slot    = 0;             // 0 = first TX/RX slot, 1 = second TX/RX slot
uint8_t RF_Channel = 0;             // hopping channel
uint8_t RF_SysID   = 0;             // current system: OGN/ADS-L/LDR

uint8_t RX_OGN_Packets=0;

static RadioEvents_t Radio_Events;

FreqPlan Radio_FreqPlan;               // RF hopping pattern

FIFO<FSK_RxPacket, 16> RxFIFO;         // buffer for received packets

Delay<uint8_t, 64> RX_OGN_CountDelay;   // to average the OGN packet rate over one minute
uint16_t           RX_OGN_Count64=0;    // counts received packets for the last 64 seconds

LowPass2<int32_t, 4,2,4> RX_RSSI;       // low pass filter to average the RX noise

LDPC_Decoder      Decoder;

bool Radio_isIdle   (void) { return Radio.GetStatus()==RF_IDLE; }
bool Radio_TxRunning(void) { return Radio.GetStatus()==RF_TX_RUNNING; }
bool Radio_RxRunning(void) { return Radio.GetStatus()==RF_RX_RUNNING; }

static void Radio_TxDone(void)  // when transmission completed
{ // Serial.printf("%d: Radio_TxDone()\n", millis());
  Radio_TxConfig(RF_SysID);
  Radio_RxConfig(RF_SysID);               // refresh the receiver configuration
  RF_Channel=Radio_FreqPlan.getChannel(GPS_PPS_UTC, RF_Slot, 1);
  Radio.RxBoosted(0); }

static void Radio_TxTimeout(void) // never happens, not clear under which conditions.
{ // Serial.printf("%d: Radio_TxTimeout()\n", millis());
  Radio_TxConfig(RF_SysID);
  Radio_RxConfig(RF_SysID);
  RF_Channel=Radio_FreqPlan.getChannel(GPS_PPS_UTC, RF_Slot, 1);
  Radio.RxBoosted(0); }

static void Radio_RxTimeout(void)                     // end-of-receive-period: not used for now
{ }

static bool Radio_CAD = 0;

static void Radio_CadDone(bool CAD)                   // when carrier sense completes
{ Radio_CAD=CAD; }

// a new packet has been received callback - this should probably be a quick call
static void Radio_RxDone( uint8_t *Packet, uint16_t Size, int16_t RSSI, int8_t SNR) // RSSI and SNR are not passed for FSK packets
{ // Serial.printf("Radio_RxDone(, %d, , ) SysID:%X Chan:%d\n", Size, RF_SysID, RF_Channel);
  uint32_t msTime=millis();
  RX_OGN_Packets++;
  PacketStatus_t RadioPktStatus; // to get the packet RSSI: https://github.com/HelTecAutomation/CubeCell-Arduino/issues/236
  SX126xGetPacketStatus(&RadioPktStatus);
  RSSI = RadioPktStatus.Params.Gfsk.RssiAvg;
  FSK_RxPacket *RxPkt = RxFIFO.getWrite();                             // new packet in the RxFIFO
  RxPkt->RSSI = -2*RSSI;                                               // [-0.5dBm]
  RxPkt->Time = GPS_PPS_UTC;                                           // [sec]
  RxPkt->msTime = msTime-GPS_PPS_ms;                                   // [ms] time since PPS
  RxPkt->Channel = RF_Channel;                                         // [ ] channel
  RxPkt->SysID = RF_SysID;
  RxPkt->Manchester = RxPkt->SysID!=Radio_SysID_LDR;                   // LDR is not Manchester encoded
  // Serial.printf("%02d.%4d: Radio_RxDone(, %d, , ) RSSI:%2d, SysID:%X Chan:%d\n",
  //         RxPkt->Time%60, RxPkt->msTime, Size, RxPkt->RSSI/2, RxPkt->SysID, RxPkt->Channel);
  if(RxPkt->Manchester)
  { RxPkt->Bytes=Size/2;
    uint8_t PktIdx=0;
    for(uint8_t Idx=0; Idx<RxPkt->Bytes; Idx++)                            // loop over packet bytes
    { uint8_t ByteH = Packet[PktIdx++];
      ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode manchester, detect (some) errors
      uint8_t ByteL = Packet[PktIdx++];
      ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F;
      RxPkt->Data[Idx]=(ByteH<<4) | ByteL;
      RxPkt->Err [Idx]=(ErrH <<4) | ErrL ; }
  } else
  { RxPkt->Bytes=Size;
    for(uint8_t Idx=0; Idx<RxPkt->Bytes; Idx++)
    { RxPkt->Data[Idx]=Packet[Idx];
      RxPkt->Err [Idx]=0; }
  }
  RxFIFO.Write();                                                      // put packet into the RxFIFO
  Random.RX = (Random.RX*RSSI) ^ (~RSSI);                              // update random number
  Random.GPS *= micros();
  XorShift64(Random.Word); }

extern SX126x_t SX126x; // access to LoraWan102 driver parameters in LoraWan102/src/radio/radio.c

// additional RF configuration reuired for OGN/ADS-L to work
static void Radio_UpdateConfig(const uint8_t *SyncWord, uint8_t SyncBytes, RadioModShapings_t BT=MOD_SHAPING_G_BT_05)
{ SX126x.ModulationParams.Params.Gfsk.ModulationShaping = BT;  // specific BT
  SX126xSetModulationParams(&SX126x.ModulationParams);
  SX126x.PacketParams.Params.Gfsk.SyncWordLength = SyncBytes*8;
  SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREE_OFF;                   //
  SX126xSetPacketParams(&SX126x.PacketParams);
  SX126xSetSyncWord((uint8_t *)SyncWord); }

void Radio_TxConfig(uint8_t SysID)
{ const uint8_t *SYNC;
  uint8_t PktLen;
  int SyncLen = FSK_RxPacket::SysSYNC(SYNC, PktLen, SysID);
  if(SysID==Radio_SysID_LDR) Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower+8, 12500, 0,  38400, 0, 5, 1, 0, 0, 0, 0, 20);
                        else Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower  , 50000, 0, 100000, 0, 1, 1, 0, 0, 0, 0, 20);
  // Modem, TxPower, Freq-dev [Hz], LoRa bandwidth, Bitrate [bps], LoRa code-rate, preamble [bytes],
  // Fixed-len [bool], CRC-on [bool], LoRa freq-hop [bool], LoRa hop-period [symbols], LoRa IQ-invert [bool], Timeout [ms]
  Radio_UpdateConfig(SYNC, SyncLen); }

void Radio_RxConfig(uint8_t SysID)
{ const uint8_t *SYNC;
  uint8_t PktLen;
  int SyncLen = FSK_RxPacket::SysSYNC(SYNC, PktLen, SysID);
  if(SysID==Radio_SysID_LDR) Radio.SetRxConfig(MODEM_FSK,  50000,  38400, 0,  50000, 4, 100, 1, PktLen  , 0, 0, 0, 0, true);
                        else Radio.SetRxConfig(MODEM_FSK, 200000, 100000, 0, 250000, 0, 100, 1, PktLen*2, 0, 0, 0, 0, true);
  // Modem, Bandwidth [Hz], Bitrate [bps], CodeRate, AFC bandwidth [Hz], preamble [bytes], Timeout [bytes], FixedLen [bool], PayloadL>
  // FreqHopOn [bool], HopPeriod, IQinvert, rxContinous [bool]
  if(SyncLen>2) Radio_UpdateConfig(SYNC+1, SyncLen-1);
          else  Radio_UpdateConfig(SYNC, SyncLen); }


#ifdef OBSOLETE

#ifdef WITH_ADSL
static void ADSL_TxConfig(void)  // RF chip config for ADS-L transmissions: identical to OGN, just different SYNC
{ Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower, 50000, 0, 100000, 0, 1, 1, 0, 0, 0, 0, 20);
  Radio_UpdateConfig(ADSL_SYNC, 8); }
#endif

#ifdef WITH_PAW
static void PAW_TxConfig(void)  // RF chip config for PilotAWare transmissions: +/-12.5kHz, 38400bps, long preamble
{ Radio.SetTxConfig(MODEM_FSK, Parameters.TxPower+8, 12500, 0, 38400, 0, 10, 1, 0, 0, 0, 0, 20);
  Radio_UpdateConfig(PAW_SYNC, 8, MOD_SHAPING_G_BT_05); }
#endif

#ifdef WITH_ADSL
static void ADSL_RxConfig(void)
{ Radio.SetRxConfig(MODEM_FSK, 200000, 100000, 0, 200000, 1, 100, 1, 48, 0, 0, 0, 0, true); // same as OGN just different packet size
  Radio_UpdateConfig(ADSL_SYNC+1, 7); }                                                       // and different SYNC
#endif

#ifdef WITH_PAW
// transmit PilotAWare packet
static int PAW_Transmit(const PAW_Packet &TxPacket)
{ PAW_TxConfig();
  uint8_t Packet[TxPacket.Size+2];
  for(uint8_t Idx=0; Idx<TxPacket.Size; Idx++)
  { Packet[Idx] = TxPacket.Byte[Idx]; }
  PAW_Packet::Whiten(Packet, TxPacket.Size);
  Packet[TxPacket.Size] = PAW_Packet::CRC8(Packet, TxPacket.Size);
  Radio.Send(Packet, TxPacket.Size+1);
  return TxPacket.Size+1; }
#endif

#endif // OBSOLETE

#ifdef WITH_MESHT
void MSH_TxConfig(void)              // setup for Meshtastic ShortFast: 250kHz bandwidth, SF7, preamble:8, sync:0x2B, explicit header,
{ Radio.SetTxConfig(MODEM_LORA, Parameters.TxPower+8, 0,     1,          7,         1,       12,              0,   1,   0, 0,          0,    100);
                 // Modem,      Power,                 , 250kHz, Data-rate, Code-rate, preanble, variable/fixed, CRC, hop,  , invert I/Q, timeout [ms]
  // uint16_t Sync = FNT_Seq>>4; Sync<<=8; Sync |= FNT_Seq&0x0F; Sync<<=4; Sync |= 0x0404;
  // Radio.SetSyncWord(Sync);
  SX126xWriteRegister(REG_LR_SYNCWORD  , 0x24);  // this should produce SYNC=0x2B for Meshtastic
  SX126xWriteRegister(REG_LR_SYNCWORD+1, 0xB4);
  // Radio.SetSyncWord(0x2B44);
}
                             // 0x002B  // SX1262 LoRa SYNC is not the same as SX127x and so there are issues
                             // With RadioLib it was possible to set the SX1262 to send 0xF1 like sx1276, but here is does not work ?

#endif

#ifdef WITH_FANET
void FNT_TxConfig(void)              // setup for FANET: 250kHz bandwidth, SF7, preamble:5, sync:0xF1, explicit header,
{ Radio.SetTxConfig(MODEM_LORA, Parameters.TxPower, 0,     1,          7,         1,        5,              0,   1,   0, 0,          0,    100);
                 // Modem,      Power,               , 250kHz, Data-rate, Code-rate, preanble, variable/fixed, CRC, hop,  , invert I/Q, timeout [ms]
  // uint16_t Sync = FNT_Seq>>4; Sync<<=8; Sync |= FNT_Seq&0x0F; Sync<<=4; Sync |= 0x0404;
  // Radio.SetSyncWord(Sync);
  SX126xWriteRegister(REG_LR_SYNCWORD  , 0xF4);  // only this method worked to set SYNC=0xF1 for FANET
  SX126xWriteRegister(REG_LR_SYNCWORD+1, 0x14);
  // Radio.SetSyncWord(0xF414);                  // attempts here did not work
}
                             // 0x00F1  // SX1262 LoRa SYNC is not the same as SX127x and so there are issues
                             // With RadioLib it was possible to set the SX1262 to send 0xF1 like sx1276, but here is does not work ?

// there is an issue with the LoRa SYNC compatibility, some research on it is here:
// https://blog.classycode.com/lora-sync-word-compatibility-between-sx127x-and-sx126x-460324d1787a
// in short: SX1262 is not able to produce exact same SYNC as SX1276 set for SYNC=0xF1 but on receive both chips are tolerant to different SYNC

void FNT_RxConfig(void)
{ Radio.SetRxConfig(MODEM_LORA,     1,          7,        4, 0,        5,               16,              0, 0,   1,   0, 0,          0,    1);
                 // Modem,     250kHz, Data-rate, Code-rate,  , preanble, RxSingle-timeout, variable/fixed, 0, CRC, hop,  , invert I/Q, continoue
  Radio.SetSyncWord(0xF144); }              // SX1262 LoRa SYNC is not the same as SX127x and so there are issues
#endif

// Manchester encode packet data
static int Manchester(uint8_t *TxData, const uint8_t *PktData, int PktLen)
{ int TxLen=0;
  for(int Idx=0; Idx<PktLen; Idx++)
  { uint8_t Byte=PktData[Idx];
    TxData[TxLen++]=ManchesterEncode[Byte>>4];                      // software manchester encode every byte
    TxData[TxLen++]=ManchesterEncode[Byte&0x0F]; }
  return TxLen; }

static uint8_t Radio_TxPacket[2*26+64+4];                                  // buffer to fit manchester encoded packet and the digital signature

// transmit Data with Manchester encoding optionally followed by a signature (without Manchester), setup already assumed done
static int ManchTransmit(const uint8_t *Data, uint8_t PktLen=26, const uint8_t *Sign=0, uint8_t SignLen=68)
{ int TxLen = Manchester(Radio_TxPacket, Data, PktLen);
  if(SignLen && Sign)
  { for(uint8_t Idx=0; Idx<SignLen; Idx++)                            // digital signature
    { uint8_t Byte=Sign[Idx];
      Radio_TxPacket[TxLen++]=Byte; }                                       // copy the bytes directly, without Manchester encoding
  }
  Radio.Send(Radio_TxPacket, TxLen);
  return TxLen; }

// transmit OGN packet with possible signature
int OGN_Transmit(const OGN_TxPacket<OGN1_Packet> &TxPacket, uint8_t *Sign, uint8_t SignLen)
{ Radio_TxConfig(Radio_SysID_OGN);
  return ManchTransmit(TxPacket.Byte(), TxPacket.Bytes, Sign, SignLen); }

#ifdef WITH_ADSL
// transmit ADS-L packet with possible signature
int ADSL_Transmit(const ADSL_Packet &TxPacket, uint8_t *Sign, uint8_t SignLen)
{ Radio_TxConfig(Radio_SysID_ADSL);
  return ManchTransmit(&(TxPacket.Version), TxPacket.TxBytes-3, Sign, SignLen); }


int LDR_Transmit(const ADSL_Packet &TxPacket)
{ Radio_TxConfig(Radio_SysID_LDR);
  int PktSize = TxPacket.TxBytes-3;
  static const uint8_t SYNC_LDR [8] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x18, 0x71 };
  memcpy(Radio_TxPacket, SYNC_LDR+2, 6);                  // first copy the remaining 6 bytes of the pre-data part
  memcpy(Radio_TxPacket+6, &(TxPacket.Version), PktSize); // copy packet to the buffer (internal CRC is already set)
  Radio_TxPacket[6+PktSize] = PAW_Packet::CRC8(Radio_TxPacket+6, PktSize); // add external CRC
  Radio.Send(Radio_TxPacket, 6+PktSize+1);
  return PktSize; }
#endif

void Radio_Init(void)
{ Radio_FreqPlan.setPlan(Parameters.FreqPlan);       // set the frequency plan from the parameters

  Radio_Events.TxDone    = Radio_TxDone;             // Radio events
  Radio_Events.TxTimeout = Radio_TxTimeout;
  Radio_Events.RxDone    = Radio_RxDone;
  Radio_Events.CadDone   = Radio_CadDone;
  Radio_Events.RxTimeout = Radio_RxTimeout;
  Radio.Init(&Radio_Events);

  Radio.SetChannel(Radio_FreqPlan.getFrequency(0));  // set on default frequency
  RF_SysID = Radio_SysID_OGN_ADSL; // Radio_SysID_OGN;
  Radio_TxConfig(Radio_SysID_OGN);
  Radio_RxConfig(Radio_SysID_OGN);
  Radio.RxBoosted(0);

  RX_RSSI.Set(-2*110);

  // Serial.println("Radio started\n");
}
