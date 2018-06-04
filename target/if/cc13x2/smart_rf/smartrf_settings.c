

//*********************************************************************************
// Generated by SmartRF Studio version 2.8.0 (build #145)
// Tested for SimpleLink SDK version: CC13x2 SDK 1.60.xx
// Device: CC1352 Rev. 1.1
//
//*********************************************************************************


//*********************************************************************************
// Parameter summary
// Address: off
// Address0: 0xAA
// Address1: 0xBB
// Frequency: 868.00000 MHz
// Data Format: Serial mode disable
// Deviation: 25.000 kHz
// Packet Length Config: Variable
// Max Packet Length: 255
// Packet Length: 30
// RX Filter BW: 110 kHz
// Symbol Rate: 50.00000 kBaud
// Sync Word Length: 32 Bits
// TX Power: 14 dBm (requires define CCFG_FORCE_VDDR_HH = 1 in ccfg.c, see CC13X2/CC26xx Technical Reference Manual)
// Whitening: No whitening


#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_mce_genfsk.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_rfe_genfsk.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_multi_protocol.h)

#include <ti/drivers/rf/RF.h>
#include "smartrf_settings.h"

const rfPowerConfig_t rfPowerTable_prop[] =
{
    {-20, 0x04C0 },
    {-15, 0x04c1 },
    {-10, 0x08C2 },
    { -5, 0x0AC4 },
    {  0, 0x0EC8 },
    {  1, 0x0EC9 },
    {  2, 0x12CA },
    {  3, 0x12CB },
    {  4, 0x16CC },
    {  5, 0x18CE },
    {  6, 0x1CD0 },
    {  7, 0x2088 },
    {  8, 0x40D6 },
    {  9, 0x38DA },
    { 10, 0x6EE1 },
    { 11, 0x5497 },
    { 12, 0x740A },
    { 13, 0xCC14 },
    { 14, 0x9F3F },
};

const uint8_t rfPowerTableSize_prop = (sizeof(rfPowerTable_prop) / sizeof(rfPowerConfig_t));

// TI-RTOS RF Mode Object
RF_Mode RF_prop =
{
    .rfMode = RF_MODE_AUTO,
    .cpePatchFxn = &rf_patch_cpe_multi_protocol, //&rf_patch_cpe_prop,
    .mcePatchFxn = &rf_patch_mce_genfsk,
    .rfePatchFxn = &rf_patch_rfe_genfsk,
};

// Overrides for CMD_PROP_RADIO_DIV_SETUP
static uint32_t pOverrides[] =
{
    // PHY: Use MCE RAM patch, RFE RAM patch
    MCE_RFE_OVERRIDE(1,0,0,1,0,0),
    // override_synth_prop_863_930_div5.xml
    // Synth: Use 48 MHz crystal as synth clock, enable extra PLL filtering
    (uint32_t)0x02400403,
    // Synth: Set minimum RTRIM to 7
    (uint32_t)0x00078793,
    // Synth: Configure extra PLL filtering
    (uint32_t)0x00108463,
    // Synth: Set Fref to 4 MHz
    (uint32_t)0x000684A3,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x0A480583,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x7AB80603,
    // override_phy_tx_pa_ramp_genfsk.xml
    // Tx: Configure PA ramping, set wait time before turning off (0x2F ticks � 16/24 us = 31.3 us).
    HW_REG_OVERRIDE(0x6028,0x002F),
    // Tx: Configure PA ramp time, PACTL2.RC=0x3 (in ADI0, set PACTL2[3]=1)
    ADI_HALFREG_OVERRIDE(0,16,0x8,0x8),
    // Tx: Configure PA ramp time, PACTL2.RC=0x3 (in ADI0, set PACTL2[4]=1)
    ADI_HALFREG_OVERRIDE(0,17,0x1,0x1),
    // override_phy_rx_aaf_bw_0xd.xml
    // Rx: Set anti-aliasing filter bandwidth to 0xD (in ADI0, set IFAMPCTL3[7:4]=0xD)
    ADI_HALFREG_OVERRIDE(0,61,0xF,0xD),
    // override_phy_rx_rssi_offset_neg2db.xml
    // Rx: Set RSSI offset to adjust reported RSSI by -2 dB
    (uint32_t)0x000288A3,
#if (CCFG_FORCE_VDDR_HH)
    // TX power override
    // DC/DC regulator: In Tx with 14 dBm PA setting, use DCDCCTL5[3:0]=0xF (DITHER_EN=1 and IPEAK=7). In Rx, use DCDCCTL5[3:0]=0xC (DITHER_EN=1 and IPEAK=4).
    (uint32_t)0xFFFC08C3,
#else
    // DC/DC regulator: In Tx, use DCDCCTL5[3:0]=0xC (DITHER_EN=1 and IPEAK=4). In Rx, use DCDCCTL5[3:0]=0xC (DITHER_EN=1 and IPEAK=4).
    (uint32_t)0xFCFC08C3,
#endif
    (uint32_t)0xFFFFFFFF,
};

// CMD_PROP_RADIO_DIV_SETUP
// Proprietary Mode Radio Setup Command for All Frequency Bands
rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup =
{
    .commandNo = 0x3807,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .modulation.modType = 0x1,
    .modulation.deviationStepSz = 0x0,
#if CC13X2_DATARATE == CC13X2_50KBPS
    .modulation.deviation = 0x64,
    .symbolRate.preScale = 0xf,
    .symbolRate.rateWord = 0x8000,
    .rxBw = 0x52,
#elif CC13X2_DATARATE == CC13X2_100KBPS
    .modulation.deviation = 0x64,
    .symbolRate.preScale = 0xF,
    .symbolRate.rateWord = 0x10000,
    .rxBw = 0x54,
#elif CC13X2_DATARATE == CC13X2_150KBPS
    .modulation.deviation = 0x64,
    .symbolRate.preScale = 0xF,
    .symbolRate.rateWord = 0x18000,
    .rxBw = 0x58,
#elif CC13X2_DATARATE == CC13X2_200KBPS
    .modulation.deviation = 0x64,
    .symbolRate.preScale = 0xF,
    .symbolRate.rateWord = 0x20000,
    .rxBw = 0x58,
#else
#error Invalid RF configuration
#endif
    .symbolRate.decimMode = 0x0,
    .preamConf.nPreamBytes = 0x4,
    .preamConf.preamMode = 0x0,
    .formatConf.nSwBits = 0x20,
    .formatConf.bBitReversal = 0x0,
    .formatConf.bMsbFirst = 0x1,
    .formatConf.fecMode = 0x0,
#if NETSTK_CFG_IEEE_802154G_EN
    .formatConf.whitenMode = 0x6,
#else
    .formatConf.whitenMode = 0x0,
#endif
    .config.frontEndMode = 0x0,
    .config.biasMode = 0x1,
    .config.analogCfgMode = 0x0,
    .config.bNoFsPowerUp = 0x0,
    .txPower = 0x740A,
    .pRegOverride = pOverrides,
    .centerFreq = 0x0364,
    .intFreq = 0x8000,
    .loDivider = 0x05,
};

// CMD_FS
// Frequency Synthesizer Programming Command
rfc_CMD_FS_t RF_cmdPropFs =
{
    .commandNo = 0x0803,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .frequency = 0x0364,
    .fractFreq = 0x0000,
    .synthConf.bTxMode = 0x0,
    .synthConf.refFreq = 0x0,
    .__dummy0 = 0x00,
    .__dummy1 = 0x00,
    .__dummy2 = 0x00,
    .__dummy3 = 0x0000,
};

// CMD_PROP_TX_ADV
rfc_CMD_PROP_TX_ADV_t RF_cmdPropTxAdv =
{
    .commandNo = 0x3803,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x00,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x00,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .pktConf.bFsOff = 0x0,       //!< \brief 0: Keep frequency synth on after command<br>
                                 //!<        1: Turn frequency synth off after command
    .pktConf.bUseCrc = 0x1,      //!< \brief 0: Do not append CRC<br>
                                 //!<        1: Append CRC
    .pktConf.bCrcIncSw = 0x0,    //!< \brief 0:Do not include sync word in CRC calculation<br>
                                 //!<        1: Include sync word in CRC calculation
    .pktConf.bCrcIncHdr = 0x0,   //!< \brief 0: Do not include header in CRC calculation<br>
                                 //!<        1: Include header in CRC calculation
#if NETSTK_CFG_IEEE_802154G_EN
    .numHdrBits = 16,          // Number of bits in header (0�32)
#else
    .numHdrBits = 8,
#endif

    .preTrigger.triggerType = TRIG_REL_START,
    .preTrigger.bEnaCmd = 0x0,
    .preTrigger.triggerNo = 0x0,
    .preTrigger.pastTrig = 1,
    .preTime = 0x00000000,

    .pktLen = 0x0000,
    .startConf.bExtTxTrig = 0x0,
    .startConf.inputMode = 0x0,
    .startConf.source = 0x0,

    .syncWord = 0x930B51DE,
    .pPkt = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
};

// CMD_PROP_RX_ADV
rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv =
{
 .commandNo = 0x3804,
  .status = 0x0000,
  .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
  .startTime = 0x00000000,
  .startTrigger.triggerType = 0x0,
  .startTrigger.bEnaCmd = 0x0,
  .startTrigger.triggerNo = 0x0,
  .startTrigger.pastTrig = 0x1,
  .condition.rule = 0x1,
  .condition.nSkip = 0x0,
  .pktConf.bFsOff = 0x0, // 0: Keep frequency synth on after command<br>
  .pktConf.bRepeatOk = 0x0,
  .pktConf.bRepeatNok = 0x0,
  .pktConf.bUseCrc =  0x1,                //!< \brief 0: Do not check CRC<br>
                                          //!<        1: Check CRC
  .pktConf.bCrcIncSw = 0x0,         //  0: Do not include sync word in CRC calculation.
                                  //    1: Include sync word in CRC calculation
  .pktConf.bCrcIncHdr = 0x0,        //  0: Do not include header in CRC calculation.
                                    //  1: Include header in CRC calculation.
  .pktConf.endType = 0x0,
  .pktConf.filterOp = 0x1,
  .rxConf.bAutoFlushIgnored = 0x0,  // If 1, automatically discard ignored packets from RX queue
  .rxConf.bAutoFlushCrcErr = 0x0,   // If 1, automatically discard packets with CRC error from RX queue
  .rxConf.bIncludeHdr = 0x1,        // If 1, include the received header or length byte in the stored packet; otherwise discard it
  .rxConf.bIncludeCrc = 0x1,        // If 1, include the received CRC field in the stored packet; otherwise discard it
  .rxConf.bAppendRssi = 0x1,        // If 1, append an RSSI byte to the packet in the RX queue
  .rxConf.bAppendTimestamp = 0x0,   // If 1, append a timestamp to the packet in the RX queue
  .rxConf.bAppendStatus = 0x0,      // If 1, append a status byte to the packet in the RX queue
  .syncWord0 = 0x930B51DE,
  .syncWord1 = 0x00000000,
  .maxPktLen = 0x7FF,               //!< \brief Packet length for fixed length, maximum packet length for variable length<br>
                                    //!<        0: Unlimited or unknown length

#if NETSTK_CFG_IEEE_802154G_EN
  .hdrConf.numHdrBits = 16,          // Number of bits in header (0�32)
  .hdrConf.numLenBits = 11,
  .lenOffset = 0xFC,
#else
  .hdrConf.numHdrBits = 0x8,
  .hdrConf.numLenBits = 0x7,
  .lenOffset = 0xFE,
#endif

  .hdrConf.lenPos = 0x0,
  .addrConf.addrType = 0x0,
  .addrConf.addrSize = 0x0,
  .addrConf.addrPos = 0x0,
  .addrConf.numAddr = 0x0,
  .endTrigger.triggerType = 0x1,
  .endTrigger.bEnaCmd = 0x0,
  .endTrigger.triggerNo = 0x0,
  .endTrigger.pastTrig = 0x1,
  .endTime = 0x00000000,
  .pAddr = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
  .pQueue = 0, // INSERT APPLICABLE POINTER: (dataQueue_t*)&xxx
  .pOutput = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
};
