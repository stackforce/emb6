/*!
 * @code
 *  ___ _____ _   ___ _  _____ ___  ___  ___ ___
 * / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 * \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 * |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 * embedded.connectivity.solutions.==============
 * @endcode
 *
 * @file       cc13x2_rf.h
 * @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
 * @author     STACKFORCE
 * @brief      This is the 6lowpan-stack driver for the rf module.
 */

/*! @defgroup emb6_if emb6 stack if driver
    This group is the if driver for the emb6 stack.
  @{  */


#ifndef CC13X2_RF_H_
#define CC13X2_RF_H_

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


#include "smart_rf/IEEE_settings.h"
#include "smart_rf/smartrf_settings.h"

/*============================================================================*/
/*                                MACROS                                      */
/*============================================================================*/

#if NETSTK_CFG_2_4_EN == 1
#define rfPowerConfig_t rfIeeePowerConfig_t
#define rfPowerTable rfPowerTable_ieee
#define rfPowerTableSize rfPowerTableSize_ieee
#elif NETSTK_CFG_2_4_EN == 0
#define rfPowerConfig_t rfPropPowerConfig_t
#define rfPowerTable rfPowerTable_prop
#define rfPowerTableSize rfPowerTableSize_prop
#endif

/* RSSI threshold for clear channel assessment. Here -100dBm */
#define RF_CCA_RSSI_THR               -90
#define RF_TX_POWER                   (TX_POWER + 130)

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


#define RX_BUFF_SIZE           NUM_DATA_ENTRIES * (MAX_DATA_LENGTH_ENTRY + RF_QUEUE_DATA_ENTRY_HEADER_SIZE + RF_QUEUE_QUEUE_ALIGN_PADDING(MAX_DATA_LENGTH_ENTRY))

/* Special type definition for power table. */
#define PACKED_TYPEDEF_CONST_STRUCT    typedef const struct __attribute__((__packed__))

/* Lowest allowed channel. */
#define CHANNEL_LOWER_LIMIT        11U
/* Highest allowed channel. */
#define CHANNEL_UPPER_LIMIT        26U

/* Carrier sense state IDLE. */
#define CCA_STATE_IDLE              0U

/* Carrier sense state BUSY. */
#define CCA_STATE_BUSY              1U

/* Events that should generate an event. */

#define RF_RX_EVENT_MASK  ( RF_EventRxOk | RF_EventRxNOk |  RF_EventRxEntryDone )

#define RF_FG_EVENT_MASK  ( RF_EventLastFGCmdDone | RF_EventFGCmdDone )

#define RF_EVENT_MASK  ( RF_EventLastCmdDone | RF_EventCmdPreempted | \
             RF_EventCmdAborted | RF_EventCmdStopped | RF_EventCmdCancelled)


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
/*                              STRUCTURES                                    */
/*============================================================================*/
typedef union {
    rfc_CMD_PROP_RADIO_DIV_SETUP_t divSetup;
    rfc_CMD_PROP_RADIO_SETUP_t propSetup;
    rfc_CMD_RADIO_SETUP_t setup;
}setupCmd_t;

typedef struct
{
    setupCmd_t *cc13x2_rf_cmdRadioSetup;
    rfc_CMD_FS_t *cc13x2_rf_cmdFs;
    RF_Mode *cc13x2_rf_RF_mode;
    rfc_CMD_IEEE_RX_t *cc13x2_rf_cmdRx;
    rfc_CMD_IEEE_TX_t *cc13x2_rf_cmdTx;
    rfc_CMD_IEEE_CCA_REQ_t *cc13x2_rf_cca;
    rfc_ieeeRxOutput_t rxStatistics;
}ieee_cmd_t;

typedef struct
{
    setupCmd_t *cc13x2_rf_cmdRadioSetup;
    rfc_CMD_FS_t *cc13x2_rf_cmdFs;
    RF_Mode *cc13x2_rf_RF_mode;
    rfc_CMD_PROP_RX_ADV_t *cc13x2_rf_cmdRx;
    rfc_CMD_PROP_TX_ADV_t *cc13x2_rf_cmdTx;
    rfc_CMD_PROP_CS_t *cc13x2_rf_cca;
    rfc_propRxOutput_t rxStatistics;
}prop_cmd_t;

/*! Storage for received packets. */
typedef struct
{
    /* Length of the received packet. */
    uint8_t len;
    /* Payload buffer for the received packet. */
    uint8_t payload[MAX_DATA_LENGTH];
    /* RSSI of the received packet. */
    int8_t  rssi;
    /* Time Stamp value  */
    uint32_t timeStamp;
    /** Pointer to the current data entry */
    rfc_dataEntryGeneral_t* currentDataEntry;
    /** Pointer to the last data entry */
    rfc_dataEntryGeneral_t* lastDataEntry;
    /* Indicates that there is an unhandled frame which must be handled in
     the event function. */
    uint8_t unhandledFrame;
    /* RF module in receiving process*/
    uint8_t is_receiving;
} rx_ctx_t;

typedef struct
{
    /* Configured transmission power. */
    int8_t txPower;
    /** flag used to mention that the radio layer is waiting for ACK after transmitting a packet */
    uint8_t TxWaitingAck;
    /** expected sequence number for the ACK */
    uint8_t expSeqNo;
    /** Time out for waiting ACK  */
    uint16_t waitForAckTimeout;
    /** flag to set when an is received */
    volatile uint8_t ackReceived;
} tx_ctx_t;

typedef union
{
    ieee_cmd_t ieeeCmd;
    prop_cmd_t propCmd;
} rf_cmd_t;


typedef struct
{
    RF_Handle rfHandle;
    RF_Object rfObject;
    RF_Params rfParams;
    RF_CmdHandle rf_cmdRXHandle;
} rf_param_t;

typedef struct
{
    rf_cmd_t    rfCmd;
    rf_param_t  rfParam;
    tx_ctx_t    txCtx;
    rx_ctx_t    rxCtx;
    uint8_t     configured;
} rf_ctx_t;


#endif /* CC13X2_RF_H_ */
