#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       cc13x2_rf.c
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      This is the 6lowpan-stack driver for the rf module.
 */

/*! @defgroup emb6_if emb6 stack if driver
    This group is the if driver for the emb6 stack.
  @{  */

/*============================================================================*/
/*                                INCLUDES                                    */
/*============================================================================*/
#include "emb6.h"
#include "target_conf.h"

#include "packetbuf.h"
#include "evproc.h"

#include <ti/drivers/rf/RF.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_ieee_cmd.h>
#include <driverlib/rf_data_entry.h>

#include "bsp.h"
/* RF register settings. */
#include "smart_rf/IEEE_settings.h"

/*! Enable or disable logging. */
#define     LOGGER_ENABLE        LOGGER_RADIO
#include    "logger.h"

/*============================================================================*/
/*                          FUNCTIONS DECLARATION                             */
/*============================================================================*/
void cc13x2_Init (void *p_netstk, e_nsErr_t *p_err);
static void cc13x2_On (e_nsErr_t *p_err);
static void cc13x2_Off (e_nsErr_t *p_err);
static void cc13x2_Send(uint8_t *p_data, uint16_t len, e_nsErr_t *p_err);
static void cc13x2_Recv(uint8_t *p_buf, uint16_t len, e_nsErr_t *p_err);
static void cc13x2_Ioctl(e_nsIocCmd_t cmd, void *p_val, e_nsErr_t *p_err);
static void loc_startRx(void);

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/
#define MAX_ADDR_SIZE 8
#define MAX_DATA_LENGTH 128

/* This is the minimum value which can be disbplayed using a signed 8 bit
 * integer =-127dBm */
#define CC13X2_MIN_RSSI            128U
#define CC13X2_MIN_RSSI_SIGNED     -128
/* Number of RSSI checks that must be lower than the defined threshold */
#define CC13X2_NUM_OFF_RSSI_CHECKS 5U

/* Power level:   5 dBm */
#define TX_POWER_5_DBM              5
/* Power level:   4 dBm */
#define TX_POWER_4_DBM              4
/* Power level:   3 dBm */
#define TX_POWER_3_DBM              3
/* Power level:   2 dBm */
#define TX_POWER_2_DBM              2
/* Power level:   1 dBm */
#define TX_POWER_1_DBM              1
/* Power level:   0 dBm */
#define TX_POWER_0_DBM              0
/* Power level:  -5 dBm */
#define TX_POWER_MINUS_5_DBM       -5
/* Power level: -10 dBm */
#define TX_POWER_MINUS_10_DBM     -10

/* Special type definition for power table. */
#define PACKED_TYPEDEF_CONST_STRUCT    typedef const struct __attribute__((__packed__))

/* Lowest allowed channel. */
#define CHANNEL_LOWER_LIMIT        11U
/* Highest allowed channel. */
#define CHANNEL_UPPER_LIMIT        26U

/* Carrier sense state IDLE. */
#define CCA_STATE_IDLE              0U

/* Events that should generate an event. */
#define RF_EVENT_MASK  ( RF_EventCmdDone | RF_EventLastCmdDone | RF_EventError | \
                         RF_EventRxAborted | RF_EventCmdAborted | RF_EventCmdStopped | \
                         RF_EventCmdCancelled | RF_EventRxOk | RF_EventInternalError | \
                         RF_EventRxNOk | RF_EventRxEntryDone | RF_EventCmdPreempted )
/*============================================================================*/
/*                                MACROS                                      */
/*============================================================================*/

/* Checks if an RF handle is valid. */
#define RF_HANDLE_IS_VALID( handle ) ( handle >= 0 )
/* Invalid RF handle. */
#define RF_INVALID_RF_HANDLE         (-1)

/* Registers a the RF event function in the event handler. */
#define RF_SEM_WAIT(_event_) evproc_regCallback(_event_, cc13x2_eventHandler)
/* Indicates an RF event. */
#define RF_SEM_POST(_event_) evproc_putEvent(E_EVPROC_HEAD, _event_, NULL)

/* TX power table calculation
 *     15..8   | 7..6 | 5..0
 *   tempCoeff |  GC  |  IB
 */
#define TX_POUT( TC, GC, IB )                                         \
  (uint16_t)((((TC) & 0xFF) << 8) | (((GC) & 0x03) << 6) | ((IB) & 0x3F))

/* Calculates the number of entries in the RF power table. */
#define NUM_TX_POWER_VALUES(x) (sizeof(x) / sizeof(txPwrVal_t))

/*============================================================================*/
/*                            ENUMERATIONS                                    */
/*============================================================================*/

/*============================================================================*/
/*                              STRUCTURES                                    */
/*============================================================================*/
/*! Storage for received packets. */
typedef struct
{
  /* Length of the received packet. */
  uint8_t len;
  /* Payload buffer for the received packet. */
  uint8_t payload[MAX_DATA_LENGTH];
  /* RSSI of the received packet. */
  int8_t  rssi;
} RxPacket;

/* Structure for power table entries. */
PACKED_TYPEDEF_CONST_STRUCT
{
  int8_t   Pout;
  uint16_t txPwrVal;
} txPwrVal_t;

/* Structure of the power table. */
PACKED_TYPEDEF_CONST_STRUCT
{
  txPwrVal_t *pTxPwrVals;
  uint8_t     numTxPwrVals;
} txPwrTbl_t;

/*============================================================================*/
/*                                  CONSTANTS                                 */
/*============================================================================*/
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN (rxBuffer, 4);
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
#endif

/* Power table values. */
const txPwrVal_t txPowerTable_ieee[] =
{ { TX_POWER_5_DBM,               TX_POUT( 0x00, 0, 0x1F ) },
  { TX_POWER_4_DBM,               TX_POUT( 0x00, 1, 0x2E ) },
  { TX_POWER_3_DBM,               TX_POUT( 0x00, 2, 0x2F ) },
  { TX_POWER_2_DBM,               TX_POUT( 0x00, 2, 0x29 ) },
  { TX_POWER_1_DBM,               TX_POUT( 0x00, 2, 0x24 ) },
  { TX_POWER_0_DBM,               TX_POUT( 0x00, 2, 0x20 ) },
  { TX_POWER_MINUS_5_DBM,         TX_POUT( 0x00, 3, 0x1C ) },
  { TX_POWER_MINUS_10_DBM,        TX_POUT( 0x00, 3, 0x12 ) } };

/* Power table. */
const txPwrTbl_t txPwrTbl_ieee[] =
    { { txPowerTable_ieee, NUM_TX_POWER_VALUES( txPowerTable_ieee ) } };

/*============================================================================*/
/*                                LOCAL VARIABLES                             */
/*============================================================================*/

/* Rx buffer includes data entry structure, hdr (len=1byte), dst addr (max of 8 bytes) and data. */
uint8_t rxBuffer[sizeof(rfc_dataEntryGeneral_t) + 1 + MAX_ADDR_SIZE + MAX_DATA_LENGTH];
/* Rx data queue. */
static dataQueue_t dataQueue;
/* Rx statistics. */
rfc_ieeeRxOutput_t rxStatistics;

//Handle for last Async command, which is needed by EasyLink_abort
static RF_CmdHandle gRxCmdHandle = RF_INVALID_RF_HANDLE;

/* RF parameter struct. */
static RF_Params gRfParams;
/* RF object. */
static RF_Object gRfObject;
/* RF handle. */
static RF_Handle gRfHandle;

static s_ns_t *gpRfNetstk;

/* Indicates that there is an unhandled frame which must be handled in
   the event function. */
bool gb_unhandledFrame;

/* Buffer for a received packet. */
static RxPacket rxPacket;

/* Configured transmission power. */
static int8_t gTxPower;

/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

/*============================================================================*/
/*                                ISR PROTOTYPES                              */
/*============================================================================*/

/*============================================================================*/
/*                              LOCAL FUNCTIONS                               */
/*============================================================================*/

/* RX event callback function passed with the RX command.
 *
 * This function handles the events generated by the RX command.
 *
 * @param rfHandle   RF handle.
 * @param cmdHandle  Command handle.
 * @param evtMask    Mask with the event that should be handled.
 * */
static void rxDoneCallback(RF_Handle rfHandle, RF_CmdHandle cmdHandle, RF_EventMask evtMask)
{
  /* Read out data entry. */
  rfc_dataEntryGeneral_t *pDataEntry;
  /* This call may be shifted since it does not make sense here. */
  pDataEntry = (rfc_dataEntryGeneral_t *)rxBuffer;

  /* Indicates if RX must be restarted. */
  bool b_restartRx = true;

  if ((evtMask & RF_EventRxOk) ||
      (evtMask & RF_EventRxNOk))
  {
#if CC13X2_RX_LED_ENABLED
     bsp_led( HAL_LED3, EN_BSP_LED_OP_BLINK );
#endif /* #if CC13X2_RX_LED_ENABLED */

    if (evtMask & RF_EventRxEntryDone)
    {
      if (!gb_unhandledFrame)
      {
        /* Store the length of the recived packet. */
        rxPacket.len = *(uint8_t *)(&pDataEntry->data);
        /* Copy the payload into RF buffer.
           The offset of 1 is to skip the length field which has already been stored. */
        memcpy(&rxPacket.payload, (&pDataEntry->data + 1), rxPacket.len);
        /* Store the RSSI of the received packet. */
        rxPacket.rssi = rxStatistics.lastRssi;
        /* Trigger the execution of the RF event callback function. */
        RF_SEM_POST(EVENT_TYPE_RF);
        /* Indicate that there is an unhandled frame. */
        gb_unhandledFrame = true;
      }
    }
  }
  else if (evtMask & (RF_EventCmdAborted | RF_EventCmdStopped |
                      RF_EventCmdPreempted | RF_EventCmdCancelled))
  {
    /* RX has been stopped.
       Do not restart. */
    b_restartRx = false;
    gRxCmdHandle = RF_INVALID_RF_HANDLE;
  }

  if( b_restartRx )
  {
    loc_startRx();
  }
}

/* This function performs the following actions:
   - Clear/reset rx parameters
   - Start reception in case it is not already running. */
static void loc_startRx( void )
{
  rfc_dataEntryGeneral_t *pDataEntry;
  RF_ScheduleCmdParams rxParam;

  /* Clear the RX buffer. */
  memset( rxBuffer, 0, sizeof(rxBuffer) );

  /* Initialize the data entry, the data queue and the RX command. */
  pDataEntry = (rfc_dataEntryGeneral_t*) rxBuffer;
  pDataEntry->length = 1 + MAX_ADDR_SIZE + MAX_DATA_LENGTH;
  pDataEntry->status = 0;
  pDataEntry->config.lenSz = 1;
  dataQueue.pCurrEntry = (uint8_t*) pDataEntry;
  dataQueue.pLastEntry = NULL;
  RF_cmdIeeeRx.pRxQ = &dataQueue;
  RF_cmdIeeeRx.pOutput = &rxStatistics;

  if( !RF_HANDLE_IS_VALID( gRxCmdHandle ) && gRfHandle )
  {
    /* Set RX parameters */
    rxParam.priority = RF_PriorityNormal;
    rxParam.endTime = 0;
    rxParam.bIeeeBgCmd = true;

    /* Clear the Rx statistics structure */
    memset(&rxStatistics, 0, sizeof(rfc_ieeeRxOutput_t));

    /* Start the reception. */
    gRxCmdHandle = RF_scheduleCmd(gRfHandle, ( RF_Op* )&RF_cmdIeeeRx,
                                  &rxParam, rxDoneCallback, RF_EVENT_MASK);
  }
}

/* Callback function registered in the emb::6 event handler.
 *
 * @parameter c_event Event that should be handled.
 * @parameter p_data  Data needed to handle the event.
 */
void cc13x2_eventHandler(c_event_t c_event, p_data_t p_data)
{
  /* Set the error code to default. */
  e_nsErr_t err = NETSTK_ERR_NONE;

  /* Check if it is the right event. */
  if (c_event == EVENT_TYPE_RF)
  {
    if (gb_unhandledFrame)
    {
      #if LOGGER_ENABLE
      uint8_t temp = rxPacket.len;
      uint8_t *p_temp = rxPacket.payload;
      /* Logging */
      LOG_RAW("==========================");
      LOG_RAW("\r\n");
      LOG_RAW("RF receive: ");
      while (temp--)
      {
        LOG_RAW("%02x ", *p_temp++);
      }
      LOG_RAW("\r\n");
      #endif

      /* Forward the call to the PHY layer. */
      gpRfNetstk->phy->recv(rxPacket.payload, rxPacket.len, &err);
      gb_unhandledFrame = false;
    }
  }
}

/* Sets the transmission power.
 *
 * @param txPower Transmission power that should be set.
 * @return        Netstack error code.
 */
static e_nsErr_t loc_setTxPower(int8_t txPower)
{
  /* Variable for the loop. */
  uint8_t i;
  /* Stores the register setting for the requested TX power. */
  uint16_t reqTxPower;
  /* RF command to set the TX power. */
  rfc_CMD_SET_TX_POWER_t rfCmd;
  /* RF status return code. */
  RF_Stat rfRetCode;
  /* Error code. */
  e_nsErr_t err;

  /* Initialize with an invalid value. */
  reqTxPower = 0xFFFF;

  /* Nothing failed yet. */
  err = NETSTK_ERR_NONE;

  /* Search for the requested TX power in the power level table. */
  for (i = 0; i < txPwrTbl_ieee->numTxPwrVals; i++)
  {
    if (txPwrTbl_ieee->pTxPwrVals[i].Pout == txPower)
    {
      /* Requested TX power found in the table. */
      reqTxPower = txPwrTbl_ieee->pTxPwrVals[i].txPwrVal;
      break;
    }
  }

  if(reqTxPower == 0xFFFF)
  {
    /* The requested TX power was not found in the
     * power table. */
    err = NETSTK_ERR_INVALID_ARGUMENT;
  }
  else
  {
    /* Setup the radio command to set the transmission power. */
    rfCmd.commandNo = CMD_SET_TX_POWER;
    memcpy(&rfCmd.txPower, &reqTxPower, sizeof(uint16_t));

    /* Assign register value from power table. */
    RF_cmdRadioSetup.txPower = reqTxPower;
    /* Store the set transmission power. */
    gTxPower = txPower;

    /* Apply the new TX power value. */
    rfRetCode = RF_runImmediateCmd(gRfHandle, (uint32_t*) &rfCmd);

    if( rfRetCode == RF_StatCmdDoneSuccess )
    {
      /* Store the set transmission power. */
      gTxPower = txPower;
    }
    else
    {
      /* Applying the new TX power value failed. */
      err = NETSTK_ERR_RF_XXX;
    }
  }

  return err;
}

/* Validates and sets the channel used to receive and transmit frames.
 *
 * @param channel Number of the channel to use.
 * @return        Netstack error code.
 */
static e_nsErr_t loc_setChannel(uint8_t channel)
{
  bool rxWasRunning;
  RF_CmdHandle rfCmdHandle;
  e_nsErr_t err;

  rxWasRunning = false;
  err = NETSTK_ERR_INVALID_ARGUMENT;

  /* Check if the requested channel is allowed. */
  if( (channel >= CHANNEL_LOWER_LIMIT) &&
      (channel <= CHANNEL_UPPER_LIMIT) )
  {
    if( RF_HANDLE_IS_VALID( gRxCmdHandle ) )
    {
      /* RX is running and will be stopped, so we need to restart it later. */
      rxWasRunning = true;

      /* RX is currently running. Stop it. */
      if(RF_cancelCmd(gRfHandle, gRxCmdHandle, 1U) == RF_StatSuccess)
      {
        /* Wait until commmand is cancelled. */
        RF_pendCmd(gRfHandle, gRxCmdHandle, (RF_EventLastCmdDone | RF_EventCmdAborted |
                                             RF_EventCmdCancelled | RF_EventCmdStopped));
      }
    }

    /* Calculate the frequency and adapt the according settings. */
    RF_cmdFs.frequency = (2405 + ( 5 * ( channel - 11 )));
    rfCmdHandle = RF_postCmd(gRfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, RF_EVENT_MASK);
    if( rfCmdHandle & ( RF_EventCmdDone | RF_EventLastCmdDone |
                        RF_EventFGCmdDone | RF_EventLastFGCmdDone) )
    {
      /* Setting the new frequency was successful.
       * Update the RX channel settings. */
      RF_cmdIeeeRx.channel = channel;
      err = NETSTK_ERR_NONE;
    }

    if(rxWasRunning)
    {
      /* RX was previously running.
       * Restart it. */
      loc_startRx();
    }
  }

  return err;
}

/* Performs clear channel assessment.
 *
 * @return Netstack error code.
 */
static e_nsErr_t loc_cca(void)
{
  /* RF status return code. */
  RF_Stat rfRetCode;
  e_nsErr_t err;

  err = NETSTK_ERR_BUSY;

  /* The IEEE CCA command requires that an RX background operation is running. */
  if(RF_HANDLE_IS_VALID(gRxCmdHandle))
  {
    /* Request the current CCA state. */
    rfRetCode = RF_runImmediateCmd(gRfHandle, (uint32_t*)&RF_cmdIeeeCca);
    if( rfRetCode == RF_StatCmdDoneSuccess )
    {
      if( CCA_STATE_IDLE == RF_cmdIeeeCca.ccaInfo.ccaState )
      {
        /* CCA result is IDLE. */
        err = NETSTK_ERR_NONE;
      }
      else
      {
        /* Channel is currently busy. */
        err = NETSTK_ERR_CHANNEL_ACESS_FAILURE;
      }
    }
  }

  return err;
}

/*============================================================================*/
/*                           API FUNCTION DEFINITIONS                         */
/*============================================================================*/

/*!
 * @brief   This function initializes the radio driver.
 *
 * @param   p_netstk Pointer to the netstack structure.
 * @param   p_err    Pointer to result enum.
 */
void cc13x2_Init (void *p_netstk, e_nsErr_t *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }

  if (p_netstk == NULL)
  {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
  #endif /* NETSTK_CFG_ARG_CHK_EN */

  /* RF command handle. */
  RF_CmdHandle rfCmdHandle;

  /* Adapt RX settings. */
  RF_cmdIeeeRx.rxConfig.bIncludePhyHdr = 0x1;
  RF_cmdIeeeRx.rxConfig.bIncludeCrc = 0x1;
  RF_cmdIeeeRx.rxConfig.bAppendRssi = 0x0;
  RF_cmdIeeeRx.rxConfig.bAppendCorrCrc = 0x0;

  /* Adapt the TX settings. */
  RF_cmdIeeeTx.txOpt.bIncludePhyHdr = 0x1;
  RF_cmdIeeeTx.txOpt.bIncludeCrc = 0x1,

  /* Register the RF event handler. */
  RF_SEM_WAIT(EVENT_TYPE_RF);

  /* Initialize variable. */
  gb_unhandledFrame = false;

  /* Set error in case something goes wrong. */
  *p_err = NETSTK_ERR_INIT;

  /* Store netstack pointer. */
  gpRfNetstk = p_netstk;

  RF_Params_init(&gRfParams);

  if(gRfHandle == NULL)
  {
    gRfHandle = RF_open(&gRfObject, &RF_prop, (RF_RadioSetup*) &RF_cmdRadioSetup , &gRfParams);
    rfCmdHandle = RF_postCmd(gRfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, RF_EVENT_MASK);
    if( rfCmdHandle & ( RF_EventCmdDone | RF_EventLastCmdDone |
                        RF_EventFGCmdDone | RF_EventLastFGCmdDone) )
    {
      loc_startRx();
      if( RF_HANDLE_IS_VALID( gRxCmdHandle ) )
      {
        /* No error. */
        *p_err = NETSTK_ERR_NONE;
      }
    }
  }
}

/*!
 * @brief   This function turns the radio transceiver on.
 *
 * @param   p_err  Pointer to result enum.
 */
static void cc13x2_On (e_nsErr_t *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }
  #endif

  *p_err = NETSTK_ERR_NONE;
}


/*!
 * @brief   This function turns the radio transceiver off.
 *
 * @param   p_err   Pointer to result enum.
 */
static void cc13x2_Off (e_nsErr_t *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }
  #endif

  if(gRfHandle != NULL)
  {
      RF_Stat stat = RF_flushCmd(gRfHandle, gRxCmdHandle, 0);
      if ( stat == RF_StatSuccess ){
          RF_close(gRfHandle);
          gRfHandle = NULL;
      }
  }
  *p_err = NETSTK_ERR_NONE;

}


/*!
 * @brief   This function transmits data.
 *
 * @param   p_data      Point to buffer storing data to send.
 * @param   len         Length of data to send.
 * @param   p_err       Pointer to result enum.
 */
static void cc13x2_Send (uint8_t      *p_data,
                         uint16_t     len,
                         e_nsErr_t    *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }

  if ((p_data == NULL) || (len == 0))
  {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
  #endif

  /* RF command handle. */
  RF_CmdHandle rfCmdHandle;

  RF_EventMask result;
  RF_ScheduleCmdParams txParam;

  *p_err = NETSTK_ERR_NONE;

  RF_cmdIeeeTx.pPayload = p_data;
  RF_cmdIeeeTx.payloadLen = len;

  #if LOGGER_ENABLE
  uint8_t temp = len;
  uint8_t *p_temp = p_data;
  /* Logging */
  LOG_RAW("\r\n");
  LOG_RAW("==========================");
  LOG_RAW("\r\n");
  LOG_RAW("RF Send: ");
  while (temp--) {
    LOG_RAW("%02x ", *p_temp++);
  }
  LOG_RAW("\r\n");
  #endif

#if CC13X2_TX_LED_ENABLED
         bsp_led( HAL_LED2, EN_BSP_LED_OP_BLINK );
#endif /* #if CC13X2_TX_LED_ENABLED */

  txParam.priority = RF_PriorityNormal;
  txParam.endTime = 0;
  txParam.bIeeeBgCmd = false;

  rfCmdHandle = RF_scheduleCmd(gRfHandle, (RF_Op*)&RF_cmdIeeeTx, &txParam, NULL, RF_EventLastFGCmdDone);
  result = RF_pendCmd(gRfHandle, rfCmdHandle,( RF_EventLastFGCmdDone | RF_EventFGCmdDone ));//| RF_EventCmdError));

  if( ! ( result & RF_EventLastFGCmdDone ) )
  {
    *p_err = NETSTK_ERR_RF_SEND;
  }

  //Temporarily to avoid losing fragments.
  bsp_delayUs(100000); //Todo Remove this delay and fix loss of fragment issue

}

/*!
 * @brief   This function receives data.
 *
 * @param   p_buf   Point to buffer storing data to send.
 * @param   len     Length of received to send.
 * @param   p_err   Pointer to result enum.
 */
static void cc13x2_Recv (uint8_t    *p_buf,
                         uint16_t   len,
                         e_nsErr_t  *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }/* if */
  #endif

}

/*!
 * @brief   Input/Output control
 *
 * @param cmd         IOCTL Command.
 * @param p_val       If wanted to assign value to a information base attribute,
 *                    p_val points to value to set.
 *                    If wanted to read value of a information base attribute,
 *                    p_val points to storing buffer.
 * @param p_err       Pointer to result enum.
 */
static void cc13x2_Ioctl (e_nsIocCmd_t    cmd,
                          void            *p_val,
                          e_nsErr_t       *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
    return;
  }
  #endif

  *p_err = NETSTK_ERR_NONE;
  switch (cmd)
  {
    case NETSTK_CMD_RF_RSSI_GET:
      *(int8_t*)p_val = rxPacket.rssi;
      break;
    case NETSTK_CMD_RF_TXPOWER_SET:
      *p_err = loc_setTxPower(*(int8_t *)p_val);
      break;
    case NETSTK_CMD_RF_TXPOWER_GET:
      *(int8_t*)p_val = gTxPower;
      break;
    case NETSTK_CMD_RF_CCA_GET:
      *p_err = loc_cca();
      break;
    case NETSTK_CMD_RF_CHAN_NUM_SET:
        *p_err = loc_setChannel(*(uint8_t*)p_val);
      break;
    case NETSTK_CMD_RF_TIMESTAMP_GET:
    case NETSTK_CMD_RF_IS_RX_BUSY:
    case NETSTK_CMD_RF_RX_PENDING:
    case NETSTK_CMD_RF_OP_MODE_SET:
    case NETSTK_CMD_RX_BUF_READ:
    case NETSTK_CMD_RF_RF_SWITCH_SET:
    case NETSTK_CMD_RF_ANT_DIV_SET:
    case NETSTK_CMD_RF_SENS_SET:
    case NETSTK_CMD_RF_SENS_GET:
    case NETSTK_CMD_RF_WOR_EN:
    default:
      /* unsupported commands are treated in same way */
      *p_err = NETSTK_ERR_CMD_UNSUPPORTED;
      break;
  }/* switch */
}

/*!
 * @brief   This function prepare the radio with a packet to be sent.
 *
 * @param   payload             Point to buffer storing data to send.
 * @param   payload_len         Length of data to send.
 * @param   p_err               Pointer to result enum.
 */
static void cc13x2_Prepare(uint8_t *payload, uint16_t payload_len, e_nsErr_t *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if ((p_err == NULL) || (payload == NULL) || (payload_len == 0))
  {
    *p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
  #endif
}


/*!
 * @brief   This function Send the packet that has previously been prepared
 *
 * @param   p_err       Pointer to result enum.
 */
static void cc13x2_Transmit(e_nsErr_t *p_err)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if (p_err == NULL)
  {
	*p_err = NETSTK_ERR_INVALID_ARGUMENT;
    return;
  }
  #endif
}

/*!
 * @brief   This function read a received packet into a buffer
 *
 * @param   buf         Point to buffer.
 * @param   buf_len     max buffer Length.
 *
 * @return  Length of received data.
 */
static uint16_t cc13x2_Read(uint8_t *buf, uint16_t buf_len)
{
  #if NETSTK_CFG_ARG_CHK_EN
  if ((buf == NULL) || (buf_len < 1))
  {
    return 0;
  }
  #endif

  return 0;
}

/*============================================================================*/
/*                             DRIVER DEFINITION                              */
/*============================================================================*/
/*! Definition of the if driver. The emb6 stack uses this structure to access
    the if module. */
const s_nsRF_t rf_driver_ticc13x2 =
{
    "CC13x2",
    cc13x2_Init,
    cc13x2_On,
    cc13x2_Off,
    cc13x2_Send,
    cc13x2_Recv,
    cc13x2_Ioctl,
    cc13x2_Prepare,
    cc13x2_Transmit,
    cc13x2_Read,
};

/*! @} 6lowpan_if */

#ifdef __cplusplus
}
#endif
