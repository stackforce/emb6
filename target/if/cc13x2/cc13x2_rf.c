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

/* BIOS Header files */
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#ifndef USE_DMM
#include <ti/drivers/rf/RF.h>
#include "Board.h"
#else
#include <dmm/dmm_rfmap.h>
#include "board.h"
#endif //USE_DMM

#include <ti/devices/DeviceFamily.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_data_entry.h>

#include "RFQueue.h"

#include "bsp.h"

#if NETSTK_CFG_2_4_EN == 1
#include "smart_rf/IEEE_settings.h"
#elif NETSTK_CFG_2_4_EN == 0
#include "smart_rf/smartrf_settings.h"
#else
#error Invalid RF configuration
#endif


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
static void loc_startRx( e_nsErr_t* p_err );

/*============================================================================*/
/*                                DEFINES                                     */
/*============================================================================*/
#define MAX_ADDR_SIZE 8
#define MAX_DATA_LENGTH 128
#define MAX_DATA_LENGTH_ENTRY 153

/* This is the minimum value which can be disbplayed using a signed 8 bit
 * integer =-127dBm */
#define CC13X2_MIN_RSSI            128U
#define CC13X2_MIN_RSSI_SIGNED     -128
/* Number of RSSI checks that must be lower than the defined threshold */
#define CC13X2_NUM_OFF_RSSI_CHECKS 5U

#define NUM_DATA_ENTRIES       4 /* NOTE: Only 4 data entries supported at the moment */


#define RX_BUFF_SIZE                          NUM_DATA_ENTRIES * (MAX_DATA_LENGTH_ENTRY + RF_QUEUE_DATA_ENTRY_HEADER_SIZE + RF_QUEUE_QUEUE_ALIGN_PADDING(MAX_DATA_LENGTH_ENTRY))

/* Special type definition for power table. */
#define PACKED_TYPEDEF_CONST_STRUCT    typedef const struct __attribute__((__packed__))

/* Lowest allowed channel. */
#define CHANNEL_LOWER_LIMIT        11U
/* Highest allowed channel. */
#define CHANNEL_UPPER_LIMIT        26U

/* Carrier sense state IDLE. */
#define CCA_STATE_IDLE              0U

union setupCmd_t{
    rfc_CMD_PROP_RADIO_DIV_SETUP_t divSetup;
    rfc_CMD_PROP_RADIO_SETUP_t propSetup;
    rfc_CMD_RADIO_SETUP_t setup;
};

/* Events that should generate an event. */

#define RF_RX_EVENT_MASK  ( RF_EventRxOk | RF_EventRxNOk |  RF_EventRxEntryDone )

#define RF_FG_EVENT_MASK  ( RF_EventLastFGCmdDone | RF_EventFGCmdDone )

#define RF_EVENT_MASK  ( RF_EventLastCmdDone | RF_EventCmdPreempted | \
             RF_EventCmdAborted | RF_EventCmdStopped | RF_EventCmdCancelled)
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

/*============================================================================*/
/*                                  CONSTANTS                                 */
/*============================================================================*/
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN (rxBuffer, 4);
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
#endif

/*============================================================================*/
/*                                LOCAL VARIABLES                             */
/*============================================================================*/

/* Rx buffer includes data entry structure, hdr (len=1byte), dst addr (max of 8 bytes) and data. */
uint8_t rxBuffer[RX_BUFF_SIZE];
/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;

#if NETSTK_CFG_2_4_EN == 1
static rfc_ieeeRxOutput_t rxStatistics;
#elif NETSTK_CFG_2_4_EN == 0
static rfc_propRxOutput_t rxStatistics;
#endif

/* RF parameter struct. */
static RF_Params rfParams;
/* RF object. */
static RF_Object rfObject;
/* RF handle. */
static RF_Handle rfHandle;

static s_ns_t *gpRfNetstk;

//Indicating that the API is initialized
static uint8_t configured = 0;

//local commands, contents will be defined by modulation type
static union setupCmd_t cc13x2_rf_cmdRadioSetup;
static rfc_CMD_FS_t cc13x2_rf_cmdFs;
static RF_Mode cc13x2_rf_RF_mode;


#if NETSTK_CFG_2_4_EN == 1
static rfc_CMD_IEEE_TX_t cc13x2_rf_cmdTx;
static rfc_CMD_IEEE_RX_t cc13x2_rf_cmdRx;
#elif NETSTK_CFG_2_4_EN == 0
static rfc_CMD_PROP_TX_ADV_t cc13x2_rf_cmdTx;
static rfc_CMD_PROP_RX_ADV_t cc13x2_rf_cmdRx;
#endif

/* Indicates that there is an unhandled frame which must be handled in
   the event function. */
bool gb_unhandledFrame;

/* Buffer for a received packet. */
static RxPacket rxPacket;

/* Configured transmission power. */
static int8_t gTxPower;

//Handle for last Async command, which is needed by loc_cmdAbort
static RF_CmdHandle rf_cmdRXHandle = RF_INVALID_RF_HANDLE;

//Callback for Async Tx complete
static void txDoneCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if ((e & RF_EventLastCmdDone | RF_EventLastFGCmdDone))
    {
#if CC13X2_TX_LED_ENABLED
         bsp_led( HAL_LED2, EN_BSP_LED_OP_BLINK );
#endif /* #if CC13X2_TX_LED_ENABLED */
        return;
    }
}

//Callback for Async Rx complete
static void rxDoneCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    /* Indicates if RX must be restarted. */
    bool b_restartRx = true;
    e_nsErr_t  p_err = NETSTK_ERR_NONE;

#if (NETSTK_CFG_2_4_EN == 1)
    if ((e & RF_EventRxOk) || (e & RF_EventRxNOk))
    {
#elif (NETSTK_CFG_2_4_EN == 0)
    rf_cmdRXHandle = RF_INVALID_RF_HANDLE;
    if ((e & RF_EventLastCmdDone) && (cc13x2_rf_cmdRx.status == PROP_DONE_OK)  && ((rxStatistics.nRxOk == 1) || (rxStatistics.nRxIgnored == 1)) )
    {
#endif

#if CC13X2_RX_LED_ENABLED
        bsp_led( HAL_LED3, EN_BSP_LED_OP_BLINK );
#endif /* #if CC13X2_RX_LED_ENABLED */

        if (!gb_unhandledFrame)
        {
            /* Read out data entry. */
            currentDataEntry = RFQueue_getDataEntry();
            /* Trigger the execution of the RF event callback function. */
            RF_SEM_POST(EVENT_TYPE_RF);

            gb_unhandledFrame = true;
        }
    }
    else if (e & (RF_EventCmdCancelled | RF_EventCmdAborted | RF_EventCmdPreempted | RF_EventCmdStopped))
    {
        p_err = NETSTK_ERR_RF_ABORTED;
        rf_cmdRXHandle = RF_INVALID_RF_HANDLE;
        b_restartRx =false;
    }
    else
    {
        p_err = NETSTK_ERR_RF_ERROR;
    }


    if( b_restartRx )
    {
      loc_startRx(&p_err);
    }
}

//*****************************************************************************
//
//! \brief Abort a previously call Async Tx/Rx.
//!
//! This function is a blocking call to abort a previous Async Tx/Rx
//!
//
//*****************************************************************************
static void loc_cmdAbort(e_nsErr_t* p_err)
{
    *p_err = NETSTK_ERR_NONE;

    if (!configured)
    {
        *p_err = NETSTK_ERR_RF_CONFIG_ERROR;
        return;
    }
    //check an Async command is running, if not return success
    if (!RF_HANDLE_IS_VALID(rf_cmdRXHandle))
    {
        return;
    }

    //force abort (gracefull param set to 0)
    if (RF_cancelCmd(rfHandle, rf_cmdRXHandle, 0) == RF_StatSuccess)
    {
        /* If command is cancelled immediately, callback may have set the cmd handle to invalid.
         * In that case, no need to pend.
         */
        if (RF_HANDLE_IS_VALID(rf_cmdRXHandle))
        {
            /* Wait for Command to complete */
            RF_EventMask result = RF_pendCmd(rfHandle, rf_cmdRXHandle, (RF_EventLastCmdDone |
                    RF_EventCmdAborted | RF_EventCmdCancelled | RF_EventCmdStopped));
            if (! (result & RF_EventLastCmdDone))
            {
                *p_err = NETSTK_ERR_RF_CMD_ERROR;
            }else
            {
                rf_cmdRXHandle = RF_INVALID_RF_HANDLE;
            }
        }else
        {
            rf_cmdRXHandle = RF_INVALID_RF_HANDLE;
        }
    }
    else
    {
        *p_err = NETSTK_ERR_RF_CMD_ERROR;
    }

    return;
}

/* Validates and sets the channel used to receive and transmit frames.
 *
 * @param channel Number of the channel to use.
 * @return        Netstack error code.
 */
static e_nsErr_t loc_setChannel(uint8_t channel)
{
    bool rxWasRunning;
    e_nsErr_t err = NETSTK_ERR_RF_ERROR;

    if ( (!configured) )
    {
        err = NETSTK_ERR_RF_CONFIG_ERROR;
        return err;
    }

      /* Check if the requested channel is allowed. */
    if( (channel >= CHANNEL_LOWER_LIMIT) &&
      (channel <= CHANNEL_UPPER_LIMIT) )
    {

#if NETSTK_CFG_2_4_EN == 1
        cc13x2_rf_cmdFs.frequency = (2405 + ( 5 * ( channel - 11 )));
        /* Update the RX channel settings. */
        cc13x2_rf_cmdRx.channel = channel;
#elif NETSTK_CFG_2_4_EN == 0
        uint16_t Cent_freq = 0x035F;
        uint16_t Frac_Freq = 0x2000;
        uint16_t Delta = 0x3333;

        /* calculate the frequency parameters for desired channel */
        Cent_freq = Cent_freq + (channel / 5);
        Frac_Freq = Frac_Freq + (channel % 5) * Delta;

        /* Set the frequency */
        cc13x2_rf_cmdFs.frequency = Cent_freq;
        cc13x2_rf_cmdFs.fractFreq = Frac_Freq;
#endif

        if (rfHandle)
        {
            if( RF_HANDLE_IS_VALID( rf_cmdRXHandle ) )
            {
            /* RX is running and will be stopped, so we need to restart it later. */
                rxWasRunning = true;
                loc_cmdAbort(&err);
            }

            /* Run command */
            RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&cc13x2_rf_cmdFs,
                RF_PriorityNormal, 0, RF_EVENT_MASK);

            if (result && ( RF_EventCmdDone | RF_EventLastCmdDone |
                    RF_EventFGCmdDone | RF_EventLastFGCmdDone) )
            {
                err = NETSTK_ERR_NONE;
            }

            if(rxWasRunning)
            {
                /* RX was previously running.
                * Restart it. */
                loc_startRx(&err);
            }
        }else
        {
            err = NETSTK_ERR_NONE;
        }
    }


    return err;
}


/* Performs clear channel assessment.
 *
 * @return Netstack error code.
 */
#if NETSTK_CFG_2_4_EN == 1
static e_nsErr_t loc_cca(void)
{
  /* RF status return code. */
  RF_Stat rfRetCode;
  e_nsErr_t err;

  err = NETSTK_ERR_BUSY;

  /* The IEEE CCA command requires that an RX background operation is running. */
  if(RF_HANDLE_IS_VALID(rf_cmdRXHandle))
  {
    /* Request the current CCA state. */
    rfRetCode = RF_runImmediateCmd(rfHandle, (uint32_t*)&RF_cmdIeeeCca);
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
#endif

static e_nsErr_t loc_setTxPower(int8_t txPower)
{
    rfc_CMD_SCH_IMM_t immOpCmd = {0};
    rfc_CMD_SET_TX_POWER_t cmdSetPower = {0};
    uint8_t txPowerIdx;
    bool rxWasRunning;

    /* Nothing failed yet. */
    e_nsErr_t err = NETSTK_ERR_NONE;

    if (!configured )
    {
        err = NETSTK_ERR_RF_CONFIG_ERROR;
        return err;
    }

    immOpCmd.commandNo = CMD_SCH_IMM;
    immOpCmd.startTrigger.triggerType = TRIG_NOW;
    immOpCmd.startTrigger.pastTrig = 1;
    immOpCmd.startTime = 0;

    cmdSetPower.commandNo = CMD_SET_TX_POWER;

    if (txPower < rfPowerTable[0].dbm)
    {
        txPower = rfPowerTable[0].dbm;
    }
    else if (txPower > rfPowerTable[rfPowerTableSize-1].dbm )
    {
        txPower = rfPowerTable[rfPowerTableSize-1].dbm;
    }

    //if max power is requested then the CCFG_FORCE_VDDR_HH must be set in
    //the ccfg
#if (NETSTK_CFG_2_4_EN == 0 && CCFG_FORCE_VDDR_HH != 0x1)
    if (txPower == rfPowerTable[rfPowerTableSize-1].dbm)
    {
        err = NETSTK_ERR_RF_ERROR;
        return err;
    }
#endif

    for (txPowerIdx = 0; txPowerIdx < rfPowerTableSize; txPowerIdx++)
    {
        if (txPower >= rfPowerTable[txPowerIdx].dbm)
        {
            cmdSetPower.txPower = rfPowerTable[txPowerIdx].txPower;
#if (NETSTK_CFG_2_4_EN == 1)
            cc13x2_rf_cmdRadioSetup.setup.txPower = rfPowerTable[txPowerIdx].txPower;
#elif (NETSTK_CFG_2_4_EN == 0)
            cc13x2_rf_cmdRadioSetup.propSetup.txPower = rfPowerTable[txPowerIdx].txPower;
#endif
        }
    }

    //point the Operational Command to the immediate set power command
    immOpCmd.cmdrVal = (uint32_t) &cmdSetPower;

    if (rfHandle)
    {
        if( RF_HANDLE_IS_VALID( rf_cmdRXHandle ) )
        {
        /* RX is running and will be stopped, so we need to restart it later. */
            rxWasRunning = true;
            loc_cmdAbort(&err);
        }

        // Send command
        RF_CmdHandle cmd = RF_postCmd(rfHandle, (RF_Op*)&immOpCmd,
                RF_PriorityNormal, 0, RF_EVENT_MASK);

        RF_EventMask result = RF_pendCmd(rfHandle, cmd, RF_EventLastCmdDone);

        if (! (result & RF_EventLastCmdDone))
        {
            err = NETSTK_ERR_RF_ERROR;
        }else
        {
            gTxPower = txPower;
        }

        if(rxWasRunning)
        {
          /* RX was previously running.
           * Restart it. */
          loc_startRx(&err);
        }
    }

    return err;
}

static e_nsErr_t  loc_getTxPower(int8_t *pi8TxPowerdBm)
{
    /* Nothing failed yet. */
    e_nsErr_t err = NETSTK_ERR_NONE;


    *pi8TxPowerdBm = gTxPower;
    return err;
}

static void loc_startRx( e_nsErr_t* p_err )
{
    //Check if not configure
    if ( !configured )
    {
        *p_err = NETSTK_ERR_RF_CONFIG_ERROR;
        return;
    }

    if( !RF_HANDLE_IS_VALID( rf_cmdRXHandle ) && rfHandle )
    {

#if  (NETSTK_CFG_2_4_EN == 1)

        RF_ScheduleCmdParams rxParam;
        /* Set RX parameters */
        rxParam.priority = RF_PriorityNormal;
        rxParam.endTime = 0;
        rxParam.bIeeeBgCmd = true;

        /* Start the reception. */
        rf_cmdRXHandle = RF_scheduleCmd(rfHandle, (RF_Op*)&cc13x2_rf_cmdRx,
                                        &rxParam, rxDoneCallback, RF_EVENT_MASK | RF_RX_EVENT_MASK);
#elif (NETSTK_CFG_2_4_EN == 0)

        //Clear the Rx statistics structure
        memset(&rxStatistics, 0, sizeof(rfc_propRxOutput_t));

        cc13x2_rf_cmdRx.startTrigger.triggerType = TRIG_NOW;
        cc13x2_rf_cmdRx.startTrigger.pastTrig = 1;
        cc13x2_rf_cmdRx.startTime = 0;

        cc13x2_rf_cmdRx.endTrigger.triggerType = TRIG_NEVER;
        cc13x2_rf_cmdRx.endTrigger.pastTrig = 1;
        cc13x2_rf_cmdRx.endTime = 0;

        rf_cmdRXHandle = RF_postCmd(rfHandle, (RF_Op*)&cc13x2_rf_cmdRx,
                                    RF_PriorityNormal, rxDoneCallback, RF_EVENT_MASK);
#endif

    }

    if ( !RF_HANDLE_IS_VALID(rf_cmdRXHandle))
    {
        *p_err = NETSTK_ERR_RF_CMD_ERROR;
    }else
    {
        *p_err = NETSTK_ERR_NONE;
    }
}


/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

/*============================================================================*/
/*                                ISR PROTOTYPES                              */
/*============================================================================*/

/*============================================================================*/
/*                              LOCAL FUNCTIONS                               */
/*============================================================================*/

/*============================================================================*/
/*                              Driver interface                              */
/*============================================================================*/

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
    /* Read out data entry. */
    while (gb_unhandledFrame && currentDataEntry->status == DATA_ENTRY_FINISHED)
    {
        uint8_t *p_data = (uint8_t*)(&currentDataEntry->data) + 1;

        /* get length of the packet in the queue
         * hdr + payload + crc + RSSI
         * */
        rxPacket.len = (uint8_t)(currentDataEntry->data);

        /* Store the RSSI of the received packet which is appended to the end of the packet in the Rx queue. */
        rxPacket.rssi = (int8_t) ((int8_t*)&(currentDataEntry->data))[rxPacket.len];

        /* Substitute the rssi length from the total length of the packet in RX queue*/
        rxPacket.len -= 1;

#if NETSTK_CFG_IEEE_802154G_EN
        uint8_t tempData;
        /* FIXME flip PHR back */
        tempData = p_data[0];
        p_data[0] = p_data[1];
        p_data[1] = tempData;
#endif
        /* Copy the payload into RF buffer.
         The offset of 1 is to skip the length field which has already been stored. */
        memcpy(&rxPacket.payload, (p_data), rxPacket.len);

        RFQueue_nextEntry();

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

        /* Read out data entry for the next iteration. */
        currentDataEntry = RFQueue_getDataEntry();
    }
    gb_unhandledFrame = false;
  }
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

  /* Register the RF event handler. */
  RF_SEM_WAIT(EVENT_TYPE_RF);

  /* Initialize variable. */
  gb_unhandledFrame = false;

  /* Set error in case something goes wrong. */
  *p_err = NETSTK_ERR_INIT;

  /* Store netstack pointer. */
  gpRfNetstk = p_netstk;

  if (configured)
  {
    loc_cmdAbort(p_err);
    if (*p_err == NETSTK_ERR_RF_CMD_ERROR)
        return;
    RF_close(rfHandle);
    rfHandle = NULL;
  }

  RF_Params_init(&rfParams);

  if(!configured)
  {
#if  (NETSTK_CFG_2_4_EN == 1)
        memcpy(&cc13x2_rf_cmdRadioSetup.setup, &RF_cmdRadioSetup, sizeof(rfc_CMD_RADIO_SETUP_t));
        memcpy(&cc13x2_rf_cmdFs, &RF_cmdIeeeFs, sizeof(rfc_CMD_FS_t));
        memcpy(&cc13x2_rf_RF_mode, &RF_ieee, sizeof(RF_Mode));
        memcpy(&cc13x2_rf_cmdRx, &RF_cmdIeeeRx, sizeof(rfc_CMD_IEEE_RX_t));
        memcpy(&cc13x2_rf_cmdTx, &RF_cmdIeeeTx, sizeof(rfc_CMD_IEEE_TX_t));
#elif (NETSTK_CFG_2_4_EN == 0)
        memcpy(&cc13x2_rf_cmdRadioSetup.divSetup, &RF_cmdPropRadioDivSetup, sizeof(rfc_CMD_PROP_RADIO_DIV_SETUP_t));
        memcpy(&cc13x2_rf_cmdFs, &RF_cmdPropFs, sizeof(rfc_CMD_FS_t));
        memcpy(&cc13x2_rf_RF_mode, &RF_prop, sizeof(RF_Mode));
        memcpy(&cc13x2_rf_cmdRx, &RF_cmdPropRxAdv, sizeof(rfc_CMD_PROP_RX_ADV_t));
        memcpy(&cc13x2_rf_cmdTx, &RF_cmdPropTxAdv, sizeof(rfc_CMD_PROP_TX_ADV_t));
#endif
  }


  /* Clear the RX buffer. */
  memset( rxBuffer, 0, sizeof(rxBuffer) );

  /* Initialize the data entry, the data queue and the RX command.*/
  if( RFQueue_defineQueue(&dataQueue,
                        rxBuffer,
                        sizeof(rxBuffer),
                        NUM_DATA_ENTRIES,
                        MAX_DATA_LENGTH_ENTRY))
  {
    /* Failed to allocate space for all data entries */
      return;
  }

#if (NETSTK_CFG_2_4_EN == 1)
  cc13x2_rf_cmdRx.pRxQ = &dataQueue;
  cc13x2_rf_cmdRx.pOutput = &rxStatistics;

  /* Read out data entry. */
  currentDataEntry = RFQueue_getDataEntry();

  if(rfHandle == NULL)
  {
     /* Request access to the radio */
     rfHandle = RF_open(&rfObject, &cc13x2_rf_RF_mode,
              (RF_RadioSetup*)&cc13x2_rf_cmdRadioSetup.setup, &rfParams);

#elif (NETSTK_CFG_2_4_EN == 0)
  cc13x2_rf_cmdRx.pQueue = &dataQueue;
  cc13x2_rf_cmdRx.pOutput = (uint8_t*) &rxStatistics;

  if(rfHandle == NULL)
  {
     /* Request access to the radio */
     rfHandle = RF_open(&rfObject, &cc13x2_rf_RF_mode,
              (RF_RadioSetup*)&cc13x2_rf_cmdRadioSetup.propSetup, &rfParams);

#endif


    //Set the frequency
    RF_CmdHandle rf_cmdHandle = RF_runCmd(rfHandle, (RF_Op*)&cc13x2_rf_cmdFs, RF_PriorityNormal, 0, //asyncCmdCallback,
            RF_EVENT_MASK);

    if( rf_cmdHandle & ( RF_EventCmdDone | RF_EventLastCmdDone |
                        RF_EventFGCmdDone | RF_EventLastFGCmdDone) )
    {
      configured = 1;
      loc_startRx(p_err);
      if( RF_HANDLE_IS_VALID( rf_cmdRXHandle ) )
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

  if(rfHandle != NULL)
  {
      loc_cmdAbort(p_err);
      RF_close(rfHandle);
      rfHandle = NULL;
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

  RF_EventMask result;
  *p_err = NETSTK_ERR_NONE;




  if (p_data!=NULL && len > 1 && configured)
  {
#if NETSTK_CFG_IEEE_802154G_EN
    /* FIXME overwrite PHR */
    uint16_t transmitLen = len - PHY_HEADER_LEN;
    uint16_t totalLen = transmitLen + packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN);
    /* FIXME only support CRC-32 */
    p_data[0] = totalLen & 0xFF;
    /* check whether the 32-bits and 16-bits CRC is used // 0x10: 16-bits CRC and 0x00 : 32-bits CRC */
    if(packetbuf_attr(PACKETBUF_ATTR_MAC_FCS_LEN) == 2)
        p_data[1] = (totalLen >> 8) + 0x10  ;
    else
        p_data[1] = (totalLen >> 8) + 0x00  ;
#else
    p_data[0] = len - PHY_HEADER_LEN + 2 ;
#endif

#if  (NETSTK_CFG_2_4_EN == 1)
    cc13x2_rf_cmdTx.pPayload = p_data;
    cc13x2_rf_cmdTx.payloadLen = len;
#elif (NETSTK_CFG_2_4_EN == 0)
    cc13x2_rf_cmdTx.pPkt = p_data;
    cc13x2_rf_cmdTx.pktLen = len;
#endif


  }
  else
  {
      *p_err = NETSTK_ERR_RF_TX_ERROR;
      return;
  }


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



#if  (NETSTK_CFG_2_4_EN == 1)
  RF_ScheduleCmdParams txParam;

  txParam.priority = RF_PriorityNormal;
  txParam.endTime = 0;
  txParam.bIeeeBgCmd = false;

  RF_CmdHandle rfCmdHandle = RF_scheduleCmd(rfHandle, (RF_Op*)&cc13x2_rf_cmdTx, &txParam, txDoneCallback, RF_EVENT_MASK | RF_FG_EVENT_MASK);
  result = RF_pendCmd(rfHandle, rfCmdHandle, RF_EVENT_MASK | RF_FG_EVENT_MASK);
  // Wait for Command to complete
  if ((result & (RF_EventCmdCancelled | RF_EventCmdAborted | RF_EventCmdPreempted | RF_EventCmdStopped)))
  {
      *p_err = NETSTK_ERR_RF_TX_ERROR;
  }

#elif  (NETSTK_CFG_2_4_EN == 0)
  loc_cmdAbort(p_err);

  cc13x2_rf_cmdTx.startTrigger.triggerType = TRIG_NOW;
  cc13x2_rf_cmdTx.startTrigger.pastTrig = 1;
  cc13x2_rf_cmdTx.startTime = 0;

  // Send packet
  RF_CmdHandle rf_cmdHandle = RF_postCmd(rfHandle, (RF_Op*)&cc13x2_rf_cmdTx,RF_PriorityHigh, txDoneCallback, RF_EVENT_MASK);
  result = RF_pendCmd(rfHandle, rf_cmdHandle, RF_EventLastCmdDone);
  // Wait for Command to complete
  if ((result & (RF_EventCmdCancelled | RF_EventCmdAborted | RF_EventCmdPreempted | RF_EventCmdStopped)))
  {
      *p_err = NETSTK_ERR_RF_TX_ERROR;
  }else
  {
      loc_startRx(p_err);
  }
#endif

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
      *p_err =  loc_setTxPower(*(int8_t *)p_val);
      break;
    case NETSTK_CMD_RF_TXPOWER_GET:
        *p_err = loc_getTxPower((int8_t*)p_val);
      break;
    case NETSTK_CMD_RF_CCA_GET:
#if  (NETSTK_CFG_2_4_EN == 1)
      *p_err = loc_cca();
#elif (NETSTK_CFG_2_4_EN == 0)
      *p_err = NETSTK_ERR_NONE;
#endif
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
