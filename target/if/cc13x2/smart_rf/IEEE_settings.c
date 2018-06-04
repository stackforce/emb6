//*********************************************************************************
// Generated by SmartRF Studio version 2.7.0 (build #23)
// Tested for SimpleLink SDK version: Template not compatible with CC13x2 SDKs
// Device: CC1352 Rev. 1.0
//
//*********************************************************************************


//*********************************************************************************
// Parameter summary
// IEEE Channel: 11
// Frequency: 2405 MHz
// SFD: 0
// Preamble (32 bit): 01010101...
// TX Power: 5 dBm (requires define CCFG_FORCE_VDDR_HH = 0 in ccfg.c, see CC13xx/CC26xx Technical Reference Manual)

#include <ti/devices/cc13x2_cc26x2/driverlib/rf_mailbox.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/rf_ieee_mailbox.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/rf_common_cmd.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/rf_ieee_cmd.h>
#include <ti/drivers/rf/RF.h>
#include <ti/devices/cc13x2_cc26x2/rf_patches/rf_patch_cpe_ieee_802_15_4.h>
#include <ti/devices/cc13x2_cc26x2/rf_patches/rf_patch_mce_ieee_802_15_4.h>
#include "IEEE_settings.h"

const rfPowerConfig_t rfPowerTable_ieee[] =
{
    {-10, 0x00D2 },
    { -5, 0x00DC },
    {  0, 0x00A0 },
    {  1, 0x00A4 },
    {  2, 0x00A9 },
    {  3, 0x00AF },
    {  4, 0x006E },
    {  5, 0x001F },
};

const uint8_t rfPowerTableSize_ieee = (sizeof(rfPowerTable_ieee) / sizeof(rfPowerConfig_t));

// TI-RTOS RF Mode Object
RF_Mode RF_ieee =
{
    .rfMode = RF_MODE_AUTO,
    .cpePatchFxn = &rf_patch_cpe_ieee_802_15_4,
    .mcePatchFxn = &rf_patch_mce_ieee_802_15_4,
    .rfePatchFxn = 0,
};

// Overrides for CMD_RADIO_SETUP
static uint32_t pOverrides[] =
{
 // override_use_patch_ieee_802_15_4.xml
 // PHY: Use MCE RAM patch, RFE ROM bank 1
 MCE_RFE_OVERRIDE(1,0,0,0,1,0),
 // override_synth_ieee_802_15_4.xml
 // Synth: Use 48 MHz crystal
 (uint32_t)0x00408403,
 // override_dcdc_rx_tx_common.xml
 // DC/DC regulator: In Tx, use DCDCCTL5[3:0]=0xC (DITHER_EN=1 and IPEAK=4). In Rx, use DCDCCTL5[3:0]=0xC (DITHER_EN=1 and IPEAK=4).
 (uint32_t)0xFCFC08C3,
 (uint32_t)0xFFFFFFFF,
};


// CMD_RADIO_SETUP
// Radio Setup Command for Pre-Defined Schemes
rfc_CMD_RADIO_SETUP_t RF_cmdRadioSetup =
{
    .commandNo = 0x0802,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .mode = 0x01,
    .loDivider = 0x00,
    .config.frontEndMode = 0x0,
    .config.biasMode = 0x0,
    .config.analogCfgMode = 0x0,
    .config.bNoFsPowerUp = 0x0,
    .txPower = 0xAB3F,
    .pRegOverride = pOverrides,
};

// CMD_FS
// Frequency Synthesizer Programming Command
rfc_CMD_FS_t RF_cmdIeeeFs =
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
    .frequency = 0x0965,
    .fractFreq = 0x0000,
    .synthConf.bTxMode = 0x1,
    .synthConf.refFreq = 0x0,
    .__dummy0 = 0x00,
    .__dummy1 = 0x00,
    .__dummy2 = 0x00,
    .__dummy3 = 0x0000,
};

// CMD_IEEE_TX
// IEEE 802.15.4 Transmit Command
rfc_CMD_IEEE_TX_t RF_cmdIeeeTx =
{
    .commandNo = 0x2C01,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .txOpt.bIncludePhyHdr = 0x1,
    .txOpt.bIncludeCrc = 0x0,
    .txOpt.payloadLenMsb = 0x0,
    .payloadLen = 0x1E,
    .pPayload = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .timeStamp = 0x00000000,
};

// CMD_IEEE_RX
// IEEE 802.15.4 Receive Command
rfc_CMD_IEEE_RX_t RF_cmdIeeeRx =
{
    .commandNo = 0x2801,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .channel = 0x1A,
    .rxConfig.bAutoFlushCrc = 0x0,
    .rxConfig.bAutoFlushIgn = 0x0,
    .rxConfig.bIncludePhyHdr = 0x1,
    .rxConfig.bIncludeCrc = 0x1,
    .rxConfig.bAppendRssi = 0x1,
    .rxConfig.bAppendCorrCrc = 0x0,
    .rxConfig.bAppendSrcInd = 0x0,
    .rxConfig.bAppendTimestamp = 0x0,
    .pRxQ = 0, // INSERT APPLICABLE POINTER: (dataQueue_t*)&xxx
    .pOutput = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .frameFiltOpt.frameFiltEn = 0x0,
    .frameFiltOpt.frameFiltStop = 0x0,
    .frameFiltOpt.autoAckEn = 0x0,
    .frameFiltOpt.slottedAckEn = 0x0,
    .frameFiltOpt.autoPendEn = 0x0,
    .frameFiltOpt.defaultPend = 0x0,
    .frameFiltOpt.bPendDataReqOnly = 0x0,
    .frameFiltOpt.bPanCoord = 0x0,
    .frameFiltOpt.maxFrameVersion = 0x3,
    .frameFiltOpt.fcfReservedMask = 0x0,
    .frameFiltOpt.modifyFtFilter = 0x0,
    .frameFiltOpt.bStrictLenFilter = 0x0,
    .frameTypes.bAcceptFt0Beacon = 0x1,
    .frameTypes.bAcceptFt1Data = 0x1,
    .frameTypes.bAcceptFt2Ack = 0x1,
    .frameTypes.bAcceptFt3MacCmd = 0x1,
    .frameTypes.bAcceptFt4Reserved = 0x1,
    .frameTypes.bAcceptFt5Reserved = 0x1,
    .frameTypes.bAcceptFt6Reserved = 0x1,
    .frameTypes.bAcceptFt7Reserved = 0x1,
    /*
     * Configure CCA options to use
     * IEEE 802.15.4 mode 3
     * and set threshold to -90 dBm.
     */
    .ccaOpt.ccaEnEnergy = 0x0,
    .ccaOpt.ccaEnCorr = 0x0,
    .ccaOpt.ccaEnSync = 0x0,
    .ccaOpt.ccaCorrOp = 0x1,
    .ccaOpt.ccaSyncOp = 0x1,
    .ccaOpt.ccaCorrThr = 0x0,
    .ccaRssiThr = 0x64,
    .__dummy0 = 0x00,
    .numExtEntries = 0x00,
    .numShortEntries = 0x00,
    .pExtEntryList = 0, // INSERT APPLICABLE POINTER: (uint32_t*)&xxx
    .pShortEntryList = 0, // INSERT APPLICABLE POINTER: (uint32_t*)&xxx
    .localExtAddr = 0x0000000000000000,
    .localShortAddr = 0x0000,
    .localPanID = 0x0000,
    .__dummy1 = 0x0000,
    .__dummy2 = 0x00,
    .endTrigger.triggerType = 0x1,
    .endTrigger.bEnaCmd = 0x0,
    .endTrigger.triggerNo = 0x0,
    .endTrigger.pastTrig = 0x0,
    .endTime = 0x00000000,
};

rfc_CMD_IEEE_CCA_REQ_t RF_cmdIeeeCca =
{
    .commandNo = 0x2403, //!<        The command ID number 0x2403
    .currentRssi = 0U,            //!<        The RSSI currently observed on the channel
    .maxRssi = 0U,                //!<        The maximum RSSI observed on the channel since Rx was started
//struct {
//   uint8_t ccaState:2;               //!< \brief Value of the current CCA state<br>
//                                     //!<        0: Idle<br>
//                                     //!<        1: Busy<br>
//                                     //!<        2: Invalid
//   uint8_t ccaEnergy:2;              //!< \brief Value of the current energy detect CCA state<br>
//                                     //!<        0: Idle<br>
//                                     //!<        1: Busy<br>
//                                     //!<        2: Invalid
//   uint8_t ccaCorr:2;                //!< \brief Value of the current correlator based carrier sense CCA state<br>
//                                     //!<        0: Idle<br>
//                                     //!<        1: Busy<br>
//                                     //!<        2: Invalid
//   uint8_t ccaSync:1;                //!< \brief Value of the current sync found based carrier sense CCA state<br>
//                                     //!<        0: Idle<br>
//                                     //!<        1: Busy
//} ccaInfo;
    .ccaInfo = 0U
};
