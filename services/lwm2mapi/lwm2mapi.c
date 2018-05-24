/*
 * --- License --------------------------------------------------------------*
 */
/*
 * emb6 is licensed under the 3-clause BSD license. This license gives everyone
 * the right to use and distribute the code, either in binary or source code
 * format, as long as the copyright license is retained in the source code.
 *
 * The emb6 is derived from the Contiki OS platform with the explicit approval
 * from Adam Dunkels. However, emb6 is made independent from the OS through the
 * removal of protothreads. In addition, APIs are made more flexible to gain
 * more adaptivity during run-time.
 *
 * The license text is:
 *
 * Copyright (c) 2015,
 * Hochschule Offenburg, University of Applied Sciences
 * Laboratory Embedded Systems and Communications Electronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * --- Module Description ---------------------------------------------------*
 */
/**
 *  \file       lwm2mapi.c
 *  \author     Institute of reliable Embedded Systems
 *              and Communication Electronics
 *  \date       $Date$
 *  \version    $Version$
 *
 *  \brief      LWM2M API.
 *
 *              The LWM2M API defines a function set on top of the serial
 *              API. It allows to control the LWM2M application layer e.g.
 *              to create, delete or access resources.
 */


/*
 * --- Includes -------------------------------------------------------------*
 */
#include "emb6.h"
#include "bsp.h"
#include "evproc.h"
#include "memb.h"
#include "lwm2mapi.h"
#include "lwm2m-device.h"
#include "lwm2m-server.h"
#include "lwm2m-engine.h"
#include "lwm2m-object.h"


#ifndef LWM2M_SERIAL_API_SUPPORT_DYN_OBJ
#define LWM2M_SERIAL_API_SUPPORT_DYN_OBJ    TRUE
#endif /* #ifndef LWM2M_SERIAL_API_SUPPORT_DYN_OBJ */

#if LWM2MAPI_PARSIFAL_OBJECTS
#include "lwm2m-objects/lwm2m-object-bme280-temp.h"
#include "lwm2m-objects/lwm2m-object-bme280-humidity.h"
#include "lwm2m-objects/lwm2m-object-bme280-airpressure.h"
#endif /* #if LWM2MAPI_PARSIFAL_OBJECTS */

#if LWM2MAPI_NIKI_EMETER_OBJECTS
#include "lwm2m-objects/lwm2m-object-ipso-voltage.h"
#include "lwm2m-objects/lwm2m-object-ipso-current.h"
#endif /* #if LWM2MAPI_NIKI_EMETER_OBJECTS */

#if LWM2MAPI_NIKI_EIS_OBJECTS
#include "lwm2m-objects/lwm2m-object-ipso-illuminance.h"
#include "lwm2m-objects/lwm2m-object-ipso-temperature.h"
#include "lwm2m-objects/lwm2m-object-ipso-humidity.h"
#include "lwm2m-objects/lwm2m-object-ipso-barometer.h"
#endif /* #if LWM2MAPI_NIKI_EIS_OBJECTS */


/*
 *  --- Macros --------------------------------------------------------------*
 */

#define LWM2M_API_SET_FIELD( dst, dstlen, type, src)       \
    do{                                                     \
      *((type*)dst) = src;                                  \
      dst += sizeof(type);                                  \
      dstlen -= sizeof(type);                               \
    } while (0);

#define LWM2M_API_SET_FIELD_MEM( dst, dstlen, src, size)   \
    do{                                                     \
      memcpy(dst, src, size);                               \
      dst += size;                                          \
      dstlen -= size;                                       \
    } while (0);

#define LWM2M_API_GET_FIELD( dst, src, srclen, type)        \
    do{                                                     \
      dst = *((type*)(src));                                \
      src += sizeof(type);                                  \
      srclen -= sizeof(type);                               \
    } while (0);

#define LWM2M_API_GET_FIELD_MEM( dst, src, srclen, size)    \
    do{                                                     \
      memcpy(dst, src, size);                               \
      src += size;                                          \
      srclen -= size;                                       \
    } while (0);


#if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE
/** Maximum number of LWM2M objects */
#ifndef LWM2MAPI_OBJ_MAX
#define LWM2MAPI_OBJ_MAX                MAX_OBJECTS
#endif /* #ifndef LWM2MAPI_OBJ_MAX */

/** Maximum number of LWM2M instances */
#ifndef LWM2MAPI_INST_MAX
#define LWM2MAPI_INST_MAX               20
#endif /* #ifndef LWM2MAPI_INST_MAX */

/** Maximum number of LWM2M resources */
#ifndef LWM2MAPI_RES_MAX
#define LWM2MAPI_RES_MAX                120
#endif /* #ifndef LWM2MAPI_RES_MAX */

/** Maximum amount of LWM2M data */
#define LWM2MAPI_DATA_MAX               (LWM2MAPI_RES_MAX * sizeof(float) * 2)
#endif /* #if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE */

/** default server ip */
#define LWM2MAPI_SERVER_IP                  0xbbbb, 0x0000, 0x0000, 0x0000, \
                                            0x0a00, 0x27ff,0xfe23, 0x6fcd

#define LWM2MAPI_SERVER_IP_CONV(dst, src)   uip_ip6addr(dst, src.u16[0], src.u16[1], src.u16[2], src.u16[3],   \
                                                             src.u16[4], src.u16[5], src.u16[6], src.u16[7])
/** default server port */
#define LWM2MAPI_SERVER_PORT                5683
/** default endpoint name */
#define LWM2MAPI_ENDPOINT                   "emb6-serialLWM2M"




/*
 *  --- Enumerations --------------------------------------------------------*
 */


/**
 * \brief   LWM2M API types.
 */
typedef enum
{
  /** Initialize the LWM2M communication. A host has to issue this command
    * before configuring/starting the LWM2M layer. */
  e_lwm2m_api_type_ret = 0x00,

  /** Set a configuration parameter. */
  e_lwm2m_api_type_cfg_set = 0x20,

  /** Get a configuration parameter. */
  e_lwm2m_api_type_cfg_get,

  /** return a configuration parameter. */
  e_lwm2m_api_type_cfg_rsp,

  /** Stop the LWM2M communication. The client tries to deregister
    * at the configured server. */
  e_lwm2m_api_type_stop = 0x30,

  /** Start the LWM2M communication. The client tries to register
    * at the configured server. */
  e_lwm2m_api_type_start,

  /** Initiate a bootstrap procedure. The client uses the bootstrap
    * configuration to connect to the bootstrap server. */
  e_lwm2m_api_type_bs = 0x33,

  /** Reset the LWM2M layer. All resources will be deleted and
    * all configurations will be reset to default. */
  e_lwm2m_api_type_reset = 0x3f,

  /** Get the Status of the LWM2M module.  */
  e_lwm2m_api_type_status_get = 0x40,

  /** Returns the status of the LWM2M module. */
  e_lwm2m_api_type_status_ret,

  /** Get the current error of the LWM2M module.  */
  e_lwm2m_api_type_error_get = 0x50,

  /** Returns the current error of the LWM2M module. */
  e_lwm2m_api_type_error_ret,

#if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE
  /** Create a LWM2M object. */
  e_lwm2m_api_type_obj_create = 0x60,

  /** Create a LWM2M object using an XML file. */
  e_lwm2m_api_type_obj_create_xml = 0x61,

  /** Returns an ID of a previously created L2M2M object. The
    * host uses this ID for further actions related to the
    * according resource (e.g. deletion). */
  e_lwm2m_api_type_obj_ret,

  /** Delete a previously created object. The host must use a
    * resource ID returned by a previous create operation. */
  e_lwm2m_api_type_obj_delete,
#endif /* #if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE */

#if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE
  /** Create a LWM2M resource. */
  e_lwm2m_api_type_res_create = 0x70,

  /** Returns an ID of a previously created LWM2M resource. The
    * host uses this ID for further actions related to the
    * according resource (e.g. deletion). */
  e_lwm2m_api_type_res_ret = 0x72,

  /** Delete a previously created resource. The host must use a
    * resource ID returned by a previous create operation. */
  e_lwm2m_api_type_res_delete,
#endif /* #if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE */

  /** Read request initiated by the device. The Device calls this
   * function whenever it receives an according request from the
   * associated LWM2M server. */
  e_lwm2m_api_type_res_rd_req = 0x80,

  /** The device/host has to answer to a LWM2M2_RES_RD_REQ using a
    * LWM2M2_RES_RD_RSP. */
  e_lwm2m_api_type_res_rd_rsp,

  /** Write request to a LWM2M resource. Both device and host use this
    * command e.g. to set configurations or stored values. */
  e_lwm2m_api_type_res_wr_req,

  /** The host/device has to answer to a LWM2M2_RES_WR_REQ using a
    * LWM2M2_RES_WR_RSP. */
  e_lwm2m_api_type_res_wr_rsp,

  /** Read request initiated by the device. The Device calls this
   * function whenever it receives an according request from the
   * associated LWM2M server. */
  e_lwm2m_api_type_inst_rd_req = 0x90,

  /** The device/host has to answer to a LWM2M2_RES_RD_REQ using a
    * LWM2M2_RES_RD_RSP. */
  e_lwm2m_api_type_inst_rd_rsp,

  /** Write request to a LWM2M instance. Both device and host use this
    * command e.g. to set configurations or stored values. */
  e_lwm2m_api_type_inst_wr_req,

  /** The host/device has to answer to a LWM2M2_INST_WR_REQ using a
    * LWM2M2_RES_WR_RSP. */
  e_lwm2m_api_type_inst_wr_rsp,

  e_lwm2m_api_type_max

} e_lwm2m_api_type_t;


/**
 * \brief   Return and status codes.
 */
typedef enum
{
  /** Describes a positive return value e.g. after
    * a command was issued. */
  e_lwm2m_api_ret_ok = 0x00,

  /** A general error occurred during the operation. */
  e_lwm2m_api_ret_error,

  /** The command is not valid or supported. */
  e_lwm2m_api_ret_error_cmd,

  /** The parameters are invalid or not supported. */
  e_lwm2m_api_ret_error_param,

  /** LWM2M module has stopped. */
  e_lwm2m_api_status_stopped = 0x30,

  /** LWM2M module has started. */
  e_lwm2m_api_status_started,

  /** The LWM2M module is registered at the server that was configured
    * or the server retrieved via bootstrapping. */
  e_lwm2m_api_status_registered,

  /** LWM2M is performing bootstrapping */
  e_lwm2m_api_status_boot,

  /** undefined status */
  e_lwm2m_api_status_undef,


} e_lwm2m_api_ret_t;


/**
 * \brief   LWM2M resource types.
 */
typedef enum
{
  /** Boolean variable type*/
  e_lwm2m_api_restype_bool = 0x00,

  /** Integer variable type*/
  e_lwm2m_api_restype_int,

  /** Float variable type*/
  e_lwm2m_api_restype_float,

  /** String variable type*/
  e_lwm2m_api_restype_str,

  /** Method type*/
  e_lwm2m_api_restype_method,

  /** undefined type */
  e_lwm2m_api_restype_undef,

} e_lwm2m_api_restype_t;


/**
 * \brief   Specific configuration IDs.
 *
 *          The CFG_GET/SET command allow setting or reading the actual
 *          configuration. Therefore specific identifiers are required.
 */
typedef enum
{
  /** IP address of the LWM2M bootstrap server. */
  e_lwm2m_api_cfgid_bssrvip,

  /** Port of the LWM2M bootstrap server. */
  e_lwm2m_api_cfgid_bssrvport,

  /** Port of the LWM2M bootstrap server. */
  e_lwm2m_api_cfgid_srvip,

  /** Port of the LWM2M bootstrap server. */
  e_lwm2m_api_cfgid_srvport,

  /** Name of the LWM2M client. The server instance will
    * use this name for identification. */
  e_lwm2m_api_cfgid_cliname,

  /** Update interval of the LWM2M client. This is used to detect
   * a disconnected device. */
  e_lwm2m_api_cfgid_updt_itv

} e_serial_api_cfgid_t;

/*
 *  --- Type Definitions -----------------------------------------------------*
 */

/** frameID */
typedef uint8_t lwm2mapi_frameID_t;

/** Get/Set ID */
typedef uint8_t lwm2mapi_cfg_getset_t;

/** Get/Set ID */
typedef uint8_t lwm2mapi_objres_crdel_t;

/** IP address address configuration*/
typedef uip_ipaddr_t lwm2mapi_cfg_ipaddr_t;

/** Port configuration */
typedef uint16_t lwm2mapi_cfg_port_t;

/** Client Name configuration */
typedef char lwm2mapi_cfg_cliname_t[LWM2M_ENDPOINT_NAME_MAX];

/** Update interval configuration */
typedef uint32_t lwm2mapi_cfg_updt_itv_t;

/** Format of a general RET response. */
typedef uint8_t lwm2mapi_ret_t;


/**
 * \brief   Definition of a callback function for commands received.
 *
 *          This function is used to define a handler used for the reception
 *          and handling of a specific command.
 *
 * \param   p_cmd   Buffer containing the command payload.
 * \param   cmdLen  Length of the command.
 * \param   Buffer for the response.
 * \param   Length of the buffer.
 *
 * \return Length of the generated response.
 */
typedef int32_t (*fn_serialApiHndl_t)( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );


/*
 *  --- Local Function Prototypes ------------------------------------------ *
 */

/** Called for LWM2M status changes.
 * For further details have a look at the function definitions. */
void _lwm2m_engine_statch_cb( uint8_t registered, void* p_data );

/** Called by the stack in case a registered event occurred.
 * For further details have a look at the function definitions. */
static void _event_callback( c_event_t ev, p_data_t data );

/** Receive data.
 * For further details have a look at the function definitions. */
static int8_t _rx_data( uint8_t* p_data, uint16_t len );

/** Generate a generic status response.
 * For further details have a look at the function definitions. */
static int32_t _rsp_status( uint8_t* p_rpl, uint16_t rplLen,
    e_lwm2m_api_type_t type, e_lwm2m_api_ret_t ret );


/** Get LWM2M path of a object/resource/instance.
 * For further details have a look at the function definitions. */
static int _get_lwm2m_url( const uint16_t* p_id0,
    const uint16_t* p_id1, const uint16_t* p_id2,
    char** url  );

/** Stop LWM2M.
 * For further details have a look at the function definitions. */
static int8_t _stopLWM2M( void );

/** Start LWM2M.
 * For further details have a look at the function definitions. */
static int8_t _startLWM2M( void );


#if (LWM2MAPI_PARSIFAL_OBJECTS || LWM2MAPI_NIKI_EMETER_OBJECTS || LWM2MAPI_NIKI_EIS_OBJECTS)
/** Callback for LWM2M instance or resource access.
 * For further details have a look at the function definitions. */
static void _lwm2m_resource_access_cb( uint16_t objID, uint16_t instId, uint16_t resID,
    uint8_t type, void* val, uint16_t len, void* p_user  );
#endif /* #if (LWM2MAPI_PARSIFAL_OBJECTS || LWM2MAPI_NIKI_EMETER_OBJECTS || LWM2MAPI_NIKI_EIS_OBJECTS) */


/** Callback function in case a CFG_GET command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_cfgSet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a PING command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_cfgGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a LWM2M_START command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_lwm2mStart( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a LWM2M_STOP command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_lwm2mStop( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a LWM2M_RESET command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_lwm2mReset( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a STATUS_GET command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_statusGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

#if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE

/** Callback function in case a OBJ_CREATE command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_obj_create( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a OBJ_DELETE command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_obj_delete( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a RES_CREATE command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_res_create( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a RES_DELETE command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_res_delete( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

#endif /* #if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE */

/** Callback function in case a RESOURCE_WRITE command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_res_wr( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

/** Callback function in case a INSTANCE_WRITE command was received. For further
 * details have a look at the function definition.*/
static int32_t _hndl_inst_wr( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen );

#if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE
/** Generic LWM2M handler to GET a resource. For furhther information
 * refer to the function definition. */
static void lwm2m_get(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user);

/** Generic LWM2M handler to PUT a resource. For furhther information
 * refer to the function definition. */
static void lwm2m_put(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user);

/** Generic LWM2M handler to POST a resource. For furhther information
 * refer to the function definition. */
static void lwm2m_post(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user);

/** Generic LWM2M handler to DELETE a resource. For furhther information
 * refer to the function definition. */
static void lwm2m_delete(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user);
#endif /* #if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE */


/** Write data to a resources depending on the command */
static int32_t _wr_res( const lwm2m_resource_t* p_lwm2mRes, uint8_t** p_cmd,
    uint16_t* p_cmdLen, uint8_t* varLen );


/*
 *  --- Local Variables ---------------------------------------------------- *
 */

/** Pointer to the TX function */
static void(*_fn_tx)(uint16_t len, void* p_param) = NULL;

/** Pointer to the Tx buffer */
static uint8_t* _p_txBuf = NULL;
/** Length of the Tx buffer */
static uint16_t _txBufLen = 0;
/** Tx parameter */
static void* _p_txParam = NULL;

/** Buffer for URLS */
static char _p_url[COAP_OBSERVER_URL_LEN];

/** LWM2M status */
static e_lwm2m_api_ret_t _status;

#if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE
/** Storage for Rest Resources */
MEMB(lwm2mrestres_storage, resource_t, LWM2MAPI_OBJ_MAX );
/** Container for dynamic objects */
static const lwm2m_object_t* lwm2m_object_container[LWM2MAPI_OBJ_MAX];
/** Storage for LWM2M Objects */
MEMB(lwm2mobject_storage, lwm2m_object_t, LWM2MAPI_OBJ_MAX );
/** Storage for LWM2M Instances */
MEMB(lwm2minstance_storage, lwm2m_instance_t, LWM2MAPI_INST_MAX);
/** Storage for LWM2M Resources */
MEMB(lwm2mresource_storage, lwm2m_resource_t, LWM2MAPI_RES_MAX);
/** Storage for LWM2M Resources */
MEMB(lwm2mdata_storage, uint8_t, LWM2MAPI_DATA_MAX );
#endif /* #if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE */


/** server IP address */
static uip_ipaddr_t _serverIP = {.u16 = {LWM2MAPI_SERVER_IP}};
/** server port */
static uint16_t _serverPort = LWM2MAPI_SERVER_PORT;
/** lwm2m endpoint name */
static lwm2mapi_cfg_cliname_t _epName = LWM2MAPI_ENDPOINT;

/** Registrated module pointer. */
static fn_lwm2m_ApiInit_t fn_lwm2mModule = NULL;


/*
 *  --- Local Functions ---------------------------------------------------- *
 */

/**
 * \brief   Callback function for periodic status check.
 *
 *          This function is called periodically to check the status
 *          of the LWM2M connection.
 *
 * \param   ev    The type of the event.
 * \param   data  Extra data.
 */
void _lwm2m_engine_statch_cb( uint8_t registered, void* p_data )
{
  int32_t ret = 0;
  uint8_t* p_txBuf = _p_txBuf;
  uint16_t txBufLen = _txBufLen;

  if( (_status != e_lwm2m_api_status_registered) &&
      lwm2m_engine_is_registered() )
  {
    /* set status as registered */
    _status = e_lwm2m_api_status_registered;
    /* Call the status get handler */
    ret = _hndl_statusGet( NULL, 0, p_txBuf, txBufLen );
  }
  else if( (_status == e_lwm2m_api_status_registered) &&
      lwm2m_engine_is_registered() == 0 )
  {
    /* set status as registered */
    _status = e_lwm2m_api_status_started;
    /* Call the status get handler */
    ret = _hndl_statusGet( NULL, 0, p_txBuf, txBufLen );
  }

  if( ret > 0 )
  {
      EMB6_ASSERT_RET( _fn_tx != NULL, );
      /* Call the associated Tx function with the according
       * parameter. */
      _fn_tx( ret, _p_txParam );
  }
}


/**
 * \brief   Callback function of the stack for new data on the RX interface.
 *
 *          This function is called by the stack everytime new data is
 *          available on the RX interface. This is required to separate
 *          Interrupts from regular operations.
 *
 * \param   ev    The type of the event.
 * \param   data  Extra data.
 */
void _event_callback( c_event_t ev, p_data_t data )
{
  int32_t ret = 0;
  uint8_t* p_txBuf = _p_txBuf;
  uint16_t txBufLen = _txBufLen;

  if( ev == EVENT_TYPE_STATUS_CHANGE )
  {
    switch( emb6_getStatus() )
    {
      case STACK_STATUS_IDLE:
      {
        /* stop LWM2M */
        _stopLWM2M();
        /* Call the status get handler */
        ret = _hndl_statusGet( NULL, 0, p_txBuf, txBufLen );
        break;
      }

      case STACK_STATUS_ACTIVE:
      {
#if LWM2M_API_AUTOSTART
        /* start LWM2M */
        _startLWM2M();
        /* Call the status get handler */
        ret = _hndl_statusGet( NULL, 0, p_txBuf, txBufLen );
#endif /* #if LWM2M_API_AUTOSTART */
        break;
      }

      case STACK_STATUS_NETWORK:
      {
        /* nothing to do */
        break;
      }

      default:
        break;
    }
  }

  if( ret > 0 )
  {
      EMB6_ASSERT_RET( _fn_tx != NULL, );
      /* Call the associated Tx function with the according
       * parameter. */
      _fn_tx( ret, _p_txParam );
  }
}


/**
 * \brief   This event is raised by the serial MAC if a frame was received.
 *
 *          Everytime a full and valid frame was received by the serial
 *          MAC it raises such an event.
 *
 * \param   p_data      Payload of the frame.
 * \param   len         Length of the frame.
 */
static int8_t _rx_data( uint8_t* p_data, uint16_t len )
{
  int8_t ret = 0;
  lwm2mapi_frameID_t id;
  size_t bufLeft = len;
  uint8_t* p_dataPtr = p_data;
  uint8_t* p_txBuf = _p_txBuf;
  uint16_t txBufLen = _txBufLen;

  fn_serialApiHndl_t f_hndl = NULL;

  /* store actual status */
  e_lwm2m_api_ret_t status = _status;

  /* A frame has been received */
  /* check parameters */
  EMB6_ASSERT_RET( (p_dataPtr != NULL), -1 );
  EMB6_ASSERT_RET( (bufLeft >= sizeof(lwm2mapi_frameID_t)), -3 );

  /* get ID */
  EMB6_ASSERT_RET( bufLeft >= sizeof(lwm2mapi_frameID_t), -2 );
  LWM2M_API_GET_FIELD( id, p_dataPtr, bufLeft, lwm2mapi_frameID_t );

  switch( id )
  {
    /* write configuration request */
    case e_lwm2m_api_type_cfg_set:
      f_hndl = _hndl_cfgSet;
      break;

    /* read configuration request */
    case e_lwm2m_api_type_cfg_get:
      f_hndl = _hndl_cfgGet;
      break;

    /* start request */
    case e_lwm2m_api_type_start:
      f_hndl = _hndl_lwm2mStart;
      break;

    /* stop request */
    case e_lwm2m_api_type_stop:
      f_hndl = _hndl_lwm2mStop;
      break;

    /* reset request */
    case e_lwm2m_api_type_reset:
      f_hndl = _hndl_lwm2mReset;
      break;

    /* read status */
    case e_lwm2m_api_type_status_get:
      f_hndl = _hndl_statusGet;
      break;

#if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE

    /* create object */
    case e_lwm2m_api_type_obj_create:
      f_hndl = _hndl_obj_create;
      break;

    /* delete object */
    case e_lwm2m_api_type_obj_delete:
      f_hndl = _hndl_obj_delete;
      break;

    /* create resource */
    case e_lwm2m_api_type_res_create:
      f_hndl = _hndl_res_create;
      break;

    /* delete resource */
    case e_lwm2m_api_type_res_delete:
      f_hndl = _hndl_res_delete;
      break;

#endif /* #if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE */

    /* write request to a resource */
    case e_lwm2m_api_type_res_wr_req:
      f_hndl = _hndl_res_wr;
      break;

    /* write request to an instance */
    case e_lwm2m_api_type_inst_wr_req:
      f_hndl = _hndl_inst_wr;
      break;

    default:
      ret = -2;
      break;
  }

  /* call the according handler */
  if( f_hndl != NULL )
  {
    ret = f_hndl( p_dataPtr, bufLeft, p_txBuf, txBufLen );
    if( ret > 0 )
    {
        EMB6_ASSERT_RET( _fn_tx != NULL, -1 );
        /* Call the associated Tx function with the according
         * parameter. */
        _fn_tx( ret, _p_txParam );
        ret = 0;
    }
  }

  if( ret != 0 )
  {
    /* The command was not found */
    EMB6_ASSERT_RET( txBufLen >= sizeof(lwm2mapi_frameID_t), -1 );
    LWM2M_API_SET_FIELD( p_txBuf, txBufLen, lwm2mapi_frameID_t,
        e_lwm2m_api_type_ret );

    switch( ret )
    {
      case -2:
        EMB6_ASSERT_RET( txBufLen >= sizeof(e_lwm2m_api_ret_t), -1 );
        LWM2M_API_SET_FIELD( p_txBuf, txBufLen, e_lwm2m_api_ret_t,
            e_lwm2m_api_ret_error_cmd );
        break;

      case -3:
        EMB6_ASSERT_RET( txBufLen >= sizeof(e_lwm2m_api_ret_t), -1 );
        LWM2M_API_SET_FIELD( p_txBuf, txBufLen, e_lwm2m_api_ret_t,
            e_lwm2m_api_ret_error_param );
        break;

      default:
        EMB6_ASSERT_RET( txBufLen >= sizeof(e_lwm2m_api_ret_t), -1 );
        LWM2M_API_SET_FIELD( p_txBuf, txBufLen, e_lwm2m_api_ret_t,
            e_lwm2m_api_ret_error );
        break;
    }

    ret = (p_txBuf - _p_txBuf);
  }
  else if( status != _status)
  {
    /* status seems to have changed */
    p_txBuf = _p_txBuf;
    txBufLen = _txBufLen;
    ret = _hndl_statusGet( NULL, 0, p_txBuf, txBufLen );

    if( ret > 0 )
      p_txBuf += ret;
  }

  if( ret > 0 )
  {
    EMB6_ASSERT_RET( _fn_tx != NULL, -1 );
    _fn_tx( (p_txBuf - _p_txBuf), _p_txParam );
    ret = 0;
  }

  return ret;
}


/*
 * \brief   Generates a generic response.
 *
 *          This generates a generic response that is used to reply
 *          to several commands.
 *
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 * \ret     Return value to use for the response
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _rsp_status( uint8_t* p_rpl, uint16_t rplLen,
   e_lwm2m_api_type_t type, e_lwm2m_api_ret_t ret )
{
 uint8_t* p_txBuf = p_rpl;

 EMB6_ASSERT_RET( p_rpl != NULL, -1 );

 /* set the according RET ID */
 EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_frameID_t), -1 );
 LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_frameID_t,
     type );

 /* set OK return value to indicate ping response */
 EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_ret_t), -1 );
 LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_ret_t,
     ret );

 return p_txBuf - p_rpl;
}


/**
 * \brief   Get Path to an LWM2M object/instance/resource
 *
 * \param   p_id0   First ID (/<p_id0>) or NULL if not requested.
 * \param   p_id1   Second ID (/<p_id0>/<p_id1>) or NULL if not requested.
 * \param   p_id2   Second ID (/<p_id0>/<p_id1>/<p_id2>) or NULL if not requested.
 * \param   url     Destination to write the url to.
 *
 * \return  Length of the URL.
 */
static int _get_lwm2m_url( const uint16_t* p_id0,
    const uint16_t* p_id1, const uint16_t* p_id2,
    char** url )
{
  EMB6_ASSERT_RET( p_id0 != NULL, 0 );
  EMB6_ASSERT_RET( url != NULL, 0 );

  int ret = 0;

  if( p_id1 == NULL )
    ret = snprintf( _p_url, COAP_OBSERVER_URL_LEN, "/%u",
        *p_id0 );
  else if( p_id2 == NULL )
    ret = snprintf( _p_url, COAP_OBSERVER_URL_LEN, "/%u/%u",
        *p_id0, *p_id1 );
  else
    ret = snprintf( _p_url, COAP_OBSERVER_URL_LEN, "/%u/%u/%u",
        *p_id0, *p_id1, *p_id2 );

  *url = _p_url;
  return ret;
}


/**
 * \brief   Stop LWM2M.
 */
static int8_t _stopLWM2M( void )
{
  int8_t ret = 0;

  /* stop the LWM2M engine */
  lwm2m_engine_stop();

  _status = e_lwm2m_api_status_stopped;
  return ret;
}


/**
 * \brief   Start LWM2M.
 */
static int8_t _startLWM2M( void )
{
    int i;
    int8_t ret = 0;
    uip_ipaddr_t tmpIP;

    LWM2MAPI_SERVER_IP_CONV( &tmpIP, _serverIP );
    lwm2m_engine_use_registration_server(1);
    lwm2m_engine_register_with_server( &tmpIP, uip_htons(_serverPort) );

    /* Initialize the OMA LWM2M engine */
    lwm2m_engine_init( _epName, _lwm2m_engine_statch_cb, NULL );

    /* register specific objects */
#if LWM2MAPI_PARSIFAL_OBJECTS
    lwm2m_object_tempBME280Init( _lwm2m_resource_access_cb, NULL );
    lwm2m_object_humidityBME280Init( _lwm2m_resource_access_cb, NULL );
    lwm2m_object_airpressureBME280Init( _lwm2m_resource_access_cb, NULL );
#endif /* #if LWM2MAPI_PARSIFAL_OBJECTS */

#if LWM2MAPI_NIKI_EMETER_OBJECTS
    lwm2m_object_ipsoVoltageInit( _lwm2m_resource_access_cb, NULL );
    lwm2m_object_ipsoCurrentInit( _lwm2m_resource_access_cb, NULL );
#endif /* LWM2MAPI_NIKI_EMETER_OBJECTS */

#if LWM2MAPI_NIKI_EIS_OBJECTS
    lwm2m_object_ipsoIlluminanceInit( _lwm2m_resource_access_cb, NULL );
    lwm2m_object_ipsoTemperatureInit( _lwm2m_resource_access_cb, NULL );
    lwm2m_object_ipsoHumidityInit( _lwm2m_resource_access_cb, NULL );
    lwm2m_object_ipsoBarometerInit( _lwm2m_resource_access_cb, NULL );
#endif /* #if LWM2MAPI_NIKI_EIS_OBJECTS */

    /* Initialize the registrated modules. */
    if( fn_lwm2mModule != NULL )
        fn_lwm2mModule();

#if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE
    /* register dynamic objects */
    for( i = 0; i < LWM2MAPI_OBJ_MAX; i++ )
    {
      if(lwm2m_object_container[i] != NULL )
        lwm2m_engine_register_object( lwm2m_object_container[i] );
    }
#endif /* #if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE */

    _status = e_lwm2m_api_status_started;
    return ret;
}


#if (LWM2MAPI_PARSIFAL_OBJECTS || LWM2MAPI_NIKI_EMETER_OBJECTS || LWM2MAPI_NIKI_EIS_OBJECTS)
/**
 * \brief   Callback issued if a LWM2M instance or resource was accessed.
 *
 * \param   objID   Object Id of the accessed resource.
 * \param   instId  Instance Id of the accessed resource.
 * \param   resID   Resource Id of the accessed resource.
 * \param   type    Type of the resource.
 * \param   val     Value that has been written.
 * \param   len     Length of the value.
 * \param   p_user  User related data.
 */
static void _lwm2m_resource_access_cb( uint16_t objID, uint16_t instId, uint16_t resID,
    uint8_t type, void* val, uint16_t len, void* p_user )
{
  int32_t ret = 0;
  uint8_t* p_txBuf = _p_txBuf;
  uint16_t txBufLen = _txBufLen;

  EMB6_ASSERT_RET( p_txBuf != NULL, );

  /* set the according RET ID */
  EMB6_ASSERT_RET( txBufLen >= sizeof(lwm2mapi_frameID_t), );
  LWM2M_API_SET_FIELD( p_txBuf, txBufLen, lwm2mapi_frameID_t,
      e_lwm2m_api_type_res_wr_req );

  objID = uip_htons( objID );
  resID = uip_htons( resID );

  /* set Object ID */
  EMB6_ASSERT_RET( txBufLen >= sizeof(uint16_t), );
  LWM2M_API_SET_FIELD( p_txBuf, txBufLen, uint16_t, objID );

  /* set Instance ID */
  EMB6_ASSERT_RET( txBufLen >= sizeof(uint8_t), );
  LWM2M_API_SET_FIELD( p_txBuf, txBufLen, uint8_t, instId );

  /* set resource ID */
  EMB6_ASSERT_RET( txBufLen >= sizeof(uint16_t), );
  LWM2M_API_SET_FIELD( p_txBuf, txBufLen, uint16_t, resID );

  if( ret == 0 )
  {
    /* resource was found. Now we can read the value from it */
    switch( type )
    {
      /* String Write */
      case LWM2M_RESOURCE_TYPE_STR_VARIABLE:
      {
        EMB6_ASSERT_RET( txBufLen >= len, );
        LWM2M_API_SET_FIELD_MEM( p_txBuf, txBufLen, val, len );
        break;
      }

      /* Integer write */
      case LWM2M_RESOURCE_TYPE_INT_VARIABLE:
      {
        int32_t tmpVal;
        EMB6_ASSERT_RET( len == sizeof(int32_t), );
        memcpy( &tmpVal, val, len );
        tmpVal = uip_htonl( tmpVal );

        EMB6_ASSERT_RET( txBufLen >= sizeof(int32_t), );
        LWM2M_API_SET_FIELD( p_txBuf, txBufLen, int32_t, tmpVal );
        break;
      }

      /* Float write */
      case LWM2M_RESOURCE_TYPE_FLOATFIX_VARIABLE:
      {
        union {
          float        f;
          unsigned int i;
        } tmpVal;

        EMB6_ASSERT_RET( len == sizeof(float), );

        memcpy( &tmpVal.i, val, len );
        tmpVal.f = (float)tmpVal.i / (float)LWM2M_FLOAT32_FRAC;
        tmpVal.i = uip_htonl( tmpVal.i );

        EMB6_ASSERT_RET( txBufLen >= sizeof(int32_t), );
        LWM2M_API_SET_FIELD( p_txBuf, txBufLen, int32_t, tmpVal.i );
        break;
      }

      /* Boolean write */
      case LWM2M_RESOURCE_TYPE_BOOLEAN_VARIABLE:
        break;

      default:
        ret = -1;
        break;
    }
  }

  if( ret == 0 )
    ret = p_txBuf - _p_txBuf;

  if( ret > 0 )
  {
      EMB6_ASSERT_RET( _fn_tx != NULL, );
      /* Call the associated Tx function with the according
       * parameter. */
      _fn_tx( ret, _p_txParam );
  }
}
#endif /* #if (LWM2MAPI_PARSIFAL_OBJECTS || LWM2MAPI_NIKI_EMETER_OBJECTS || LWM2MAPI_NIKI_EIS_OBJECTS) */


/**
 * \brief   Callback to set LWM2M parameters
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_cfgSet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret = 0;
  uint8_t* p_data = p_cmd;
  int32_t _updtItv = 0;
  int i;

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  EMB6_ASSERT_RET( _status == e_lwm2m_api_status_stopped, -1 );

  /* get the type of configuration */
  lwm2mapi_cfg_getset_t cfgsetId;
  EMB6_ASSERT_RET( cmdLen >= sizeof(lwm2mapi_cfg_getset_t), -3 );
  LWM2M_API_GET_FIELD( cfgsetId, p_data, cmdLen, lwm2mapi_cfg_getset_t );

  switch( cfgsetId )
  {
    case e_lwm2m_api_cfgid_srvip:

      /* set the configuration value */
      EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_cfg_ipaddr_t), -1 );
      for( i = 0; i < (sizeof(_serverIP ) / sizeof(uint16_t)); i++ )
      {
        LWM2M_API_GET_FIELD( _serverIP.u16[i], p_data,
            cmdLen, uint16_t );

        _serverIP.u16[i] = uip_ntohs(_serverIP.u16[i]);
      }
      break;

    /* Server port shall be set */
    case e_lwm2m_api_cfgid_srvport:

      /* get the configuration value */
      EMB6_ASSERT_RET( (cmdLen >= sizeof(lwm2mapi_cfg_port_t)), -3 );
      LWM2M_API_GET_FIELD( _serverPort, p_data,
          cmdLen, lwm2mapi_cfg_port_t );

      _serverPort = uip_ntohs( _serverPort );
      break;

    /* Client name shall be set  */
    case e_lwm2m_api_cfgid_cliname:

      /* set the configuration value */
      EMB6_ASSERT_RET( cmdLen <= sizeof(lwm2mapi_cfg_cliname_t), -3 );
      EMB6_ASSERT_RET( cmdLen > 0, -3 );

      memset( _epName, 0, sizeof(_epName) );
      LWM2M_API_GET_FIELD_MEM( _epName, p_data, cmdLen,
          cmdLen );
      break;

    case e_lwm2m_api_cfgid_updt_itv:

      /* get the configuration value */
      EMB6_ASSERT_RET( (cmdLen >= sizeof(lwm2mapi_cfg_updt_itv_t)), -3 );
      LWM2M_API_GET_FIELD( _updtItv, p_data,
          cmdLen, lwm2mapi_cfg_updt_itv_t );

      _updtItv = uip_htonl( _updtItv );
      lwm2m_server_setLifetime( _updtItv );
      break;

    default:
      ret = -3;
      break;
  }

  if( ret == 0 )
  {
    EMB6_ASSERT_RET( p_rpl != NULL, -1 );
    ret = _rsp_status( p_rpl, rplLen, e_lwm2m_api_type_ret,
        e_lwm2m_api_ret_ok );
  }

  return ret;
}


/**
 * \brief   Callback to get LWM2M parameters
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_cfgGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret = 0;
  uint8_t* p_data = p_cmd;
  uint8_t* p_txBuf = p_rpl;
  int i;

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  /* get the type of configuration */
  lwm2mapi_cfg_getset_t cfgsetId;
  EMB6_ASSERT_RET( cmdLen >= sizeof(lwm2mapi_cfg_getset_t), -3 );
  LWM2M_API_GET_FIELD( cfgsetId, p_data, cmdLen, lwm2mapi_cfg_getset_t );

  /* set type of the response frame */
  EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_frameID_t), -1 );
  LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_frameID_t,
      e_lwm2m_api_type_cfg_rsp);

  /* set ID of the response frame */
  EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_cfg_getset_t), -1 );
  LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_cfg_getset_t,
      cfgsetId);

  switch( cfgsetId )
  {
    case e_lwm2m_api_cfgid_srvip:

      /* set the configuration value */
      EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_cfg_ipaddr_t), -1 );
      for( i = 0; i < (sizeof(_serverIP ) / sizeof(uint16_t)); i++ )
      {
        LWM2M_API_SET_FIELD( p_txBuf, rplLen, uint16_t,
            uip_htons( _serverIP.u16[i] ) );
      }
      break;

    case e_lwm2m_api_cfgid_srvport:

      /* set the configuration value */
      EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_cfg_port_t), -1 );
      LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_cfg_port_t,
          uip_htons( _serverPort ) );
      break;

    case e_lwm2m_api_cfgid_cliname:

      /* set the configuration value */
      EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_cfg_cliname_t), -1 );
      LWM2M_API_SET_FIELD_MEM( p_txBuf, rplLen, _epName,
          strlen( _epName ) );
      break;

    case e_lwm2m_api_cfgid_updt_itv:

      /* set the configuration value */
      EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_cfg_updt_itv_t), -1 );
      LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_cfg_updt_itv_t,
          uip_htonl( lwm2m_server_getLifetime() ) );
      break;

    default:
      ret = -3;
      break;
  }

  if( ret == 0 )
    ret = p_txBuf - p_rpl;

  return ret;
}


/**
 * \brief   Callback function for a LWM2M_START command.
 *
 *          This function is called whenever a
 *          LWM2M_START command was received
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_lwm2mStart( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret;
  e_lwm2m_api_ret_t e_ret = e_lwm2m_api_ret_ok;

  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  if( (_status != e_lwm2m_api_status_started) ||
      (_status != e_lwm2m_api_status_registered))
    _startLWM2M();

  /* set the according status */
  EMB6_ASSERT_RET( p_rpl != NULL, -1 );
  ret = _rsp_status( p_rpl, rplLen, e_lwm2m_api_type_ret,
      e_ret );

  return ret;
}


/**
 * \brief   Callback function for a LWM2M_STOP command.
 *
 *          This function is called whenever a
 *          LWM2M_STOP command was received
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_lwm2mStop( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret;
  e_lwm2m_api_ret_t e_ret = e_lwm2m_api_ret_ok;

  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  if( _status != e_lwm2m_api_status_stopped )
    _stopLWM2M();

  /* set the according status */
  EMB6_ASSERT_RET( p_rpl != NULL, -1 );
  ret = _rsp_status( p_rpl, rplLen, e_lwm2m_api_type_ret,
      e_ret );

  return ret;
}


/**
 * \brief   Callback function for a LWM2M_RESET command.
 *
 *          This function is called whenever a
 *          LWM2M_RESET command was received
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_lwm2mReset( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret;
  e_lwm2m_api_ret_t e_ret = e_lwm2m_api_ret_ok;

  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  if( _status != e_lwm2m_api_status_stopped )
    _stopLWM2M();

  /* Initialize layer */
  lwm2mApiInit(_p_txBuf, _txBufLen, _fn_tx, _p_txParam);

  /* set the according status */
  EMB6_ASSERT_RET( p_rpl != NULL, -1 );
  ret = _rsp_status( p_rpl, rplLen, e_lwm2m_api_type_ret,
      e_ret );

  return ret;
}


/**
 * \brief   Callback function for XXX commands.
 *
 *          This function is called whenever a XXX
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_statusGet( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret;
  e_lwm2m_api_ret_t e_ret;

  EMB6_ASSERT_RET( p_rpl != NULL, 0 );

  if( (_status < e_lwm2m_api_status_stopped) ||
      (_status > e_lwm2m_api_status_boot))
    e_ret = e_lwm2m_api_status_undef;
  else
    e_ret = _status;

  /* set the according status */
  EMB6_ASSERT_RET( p_rpl != NULL, -1 );
  ret = _rsp_status( p_rpl, rplLen, e_lwm2m_api_type_status_ret,
      e_ret );

  return ret;
}

#if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE

/**
 * \brief   Callback to create a LWM2M object
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_obj_create( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int i = 0;
  int32_t ret = -1;
  uint8_t* p_data = p_cmd;
  uint8_t* p_txBuf = p_rpl;

  int16_t objId;
  int8_t instId;

  const lwm2m_object_t* p_lwm2mObj = NULL;
  lwm2m_instance_t* p_lwm2mInst = NULL;

  lwm2m_object_t* p_lwm2mObjTmp = NULL;
  lwm2m_instance_t* p_lwm2mInstTmp = NULL;
  resource_t* p_restTmp = NULL;
  uint8_t objNew = FALSE;

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );
  EMB6_ASSERT_RET( _status == e_lwm2m_api_status_stopped, -1 );

  /* get the object ID and instance ID */
  EMB6_ASSERT_RET( cmdLen >= sizeof(uint16_t), -2 );
  LWM2M_API_GET_FIELD( objId, p_data, cmdLen, uint16_t );
  EMB6_ASSERT_RET( cmdLen >= sizeof(uint8_t), -2 );
  LWM2M_API_GET_FIELD( instId, p_data, cmdLen, uint8_t );

  objId = uip_ntohs( objId );

  /* try to get the according object */
  for( i = 0; i < LWM2MAPI_OBJ_MAX; i++ )
  {
    if((lwm2m_object_container[i] != NULL) &&
        (lwm2m_object_container[i]->id == objId) )
    {
      p_lwm2mObj = lwm2m_object_container[i];
      break;
    }
  }

  if( p_lwm2mObj != NULL )
  {
    /* object already exists ... check for the instance */
    p_lwm2mInst = p_lwm2mObj->p_instances;
    for( i = 0; (i < p_lwm2mObj->count) && (p_lwm2mInst != NULL); i++ )
    {
      if( p_lwm2mInst->id == instId )
        /* we found the according instance ID */
        break;
      p_lwm2mInst = p_lwm2mInst->p_next;
    }

    if( (i >= p_lwm2mObj->count) || (p_lwm2mInst == NULL) )
      /* instance was not found */
      p_lwm2mInst = NULL;
  }

  if( p_lwm2mObj == NULL )
  {
    objNew = TRUE;

    /* allocate memory for an object */
    p_lwm2mObjTmp = (lwm2m_object_t* )memb_alloc( &lwm2mobject_storage );

    if( p_lwm2mObjTmp != NULL )
    {
      /* allocate memory for a resource */
      p_restTmp = (resource_t* )memb_alloc( &lwm2mrestres_storage );
      if( p_restTmp != NULL )
      {
        *p_restTmp = (resource_t){NULL, NULL, HAS_SUB_RESOURCES | IS_OBSERVABLE, NULL,
          lwm2m_get, lwm2m_post, lwm2m_put, lwm2m_delete, {NULL}, p_lwm2mObjTmp };
        *p_lwm2mObjTmp = (lwm2m_object_t){ objId, 0, {0}, p_restTmp, NULL};
        snprintf( p_lwm2mObjTmp->path, 6, "%d", objId );
        p_lwm2mObj = p_lwm2mObjTmp;
      }
    }
  }

  if( p_lwm2mObj != NULL )
  {
    if( p_lwm2mInst == NULL )
    {
      /* allocate memory for an instance */
      p_lwm2mInstTmp = (lwm2m_instance_t*)memb_alloc( &lwm2minstance_storage );
      p_lwm2mInst = p_lwm2mInstTmp;

      if( p_lwm2mInst != NULL )
      {
        /* Initialize instance */
        *p_lwm2mInst = (lwm2m_instance_t) {instId, 0, LWM2M_INSTANCE_FLAG_USED,NULL, NULL};

        /* add instance to the object */
        lwm2m_instance_t* p_i = p_lwm2mObj->p_instances;
        while( (p_i != NULL) && (p_i->p_next != NULL) )
          p_i = p_i->p_next;

        if( p_i == NULL )
          ((lwm2m_object_t*)(p_lwm2mObj))->p_instances = p_lwm2mInst;
        else
          p_i->p_next = p_lwm2mInst;
        ((lwm2m_object_t*)(p_lwm2mObj))->count++;
      }
    }

    if( p_lwm2mInst != NULL )
    {
      ret = 0;
    }
  }

  /* set type of the response frame */
  EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_frameID_t), -1 );
  LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_frameID_t,
     e_lwm2m_api_type_obj_ret);

  /* set Object ID of the response frame */
  EMB6_ASSERT_RET( rplLen >= sizeof(uint16_t), -1 );
  LWM2M_API_SET_FIELD( p_txBuf, rplLen, uint16_t, uip_ntohs( objId ));

  /* set instance ID of the response frame */
  EMB6_ASSERT_RET( rplLen >= sizeof(uint8_t), -1 );
  LWM2M_API_SET_FIELD( p_txBuf, rplLen, uint8_t,  instId );

  if( ret == 0 )
  {
    if( objNew == TRUE )
    {
      /* add object to container only if not there yet */
      for( i = 0; i < LWM2MAPI_OBJ_MAX; i++ )
      {
        if(lwm2m_object_container[i] == NULL )
        {
          lwm2m_object_container[i] = p_lwm2mObj;
          break;
        }
      }

      /* link object to LWM2M engine */
      lwm2m_engine_register_object( p_lwm2mObj );
    }

    /* set status OK */
    EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_ret_t), -1 );
    LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_ret_t, e_lwm2m_api_ret_ok );
  }
  else
  {
    if( p_lwm2mObjTmp != NULL )
      memb_free( &lwm2mobject_storage, p_lwm2mObjTmp );
    if( p_lwm2mInstTmp != NULL )
      memb_free( &lwm2minstance_storage, p_lwm2mInstTmp );
    if( p_restTmp != NULL )
      memb_free( &lwm2mrestres_storage, p_restTmp );

    /* set status failure */
    EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_ret_t), -1 );
    LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_ret_t, e_lwm2m_api_ret_error );
  }

  ret = p_txBuf - p_rpl;
  return ret;
}


/**
 * \brief   Callback to delete a LWM2M object
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_obj_delete( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
#if 0
  int32_t ret = 0;
  uint8_t* p_data = p_cmd;
  uint8_t* p_txBuf = p_rpl;
  int i;
#endif

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );
  EMB6_ASSERT_RET( _status == e_lwm2m_api_status_stopped, -1 );

  return -1;
}


/**
 * \brief   Callback to create a LWM2M resource
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_res_create( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int i;
  int32_t ret = -1;
  uint8_t* p_data = p_cmd;
  uint8_t* p_txBuf = p_rpl;

  int16_t objId;
  int8_t instId;
  int16_t resId;
  uint8_t type;
  uint8_t varLen;
  uint8_t access;

  const lwm2m_object_t* p_lwm2mObj = NULL;
  lwm2m_instance_t* p_lwm2mInst = NULL;

  lwm2m_resource_t* p_res = NULL;
  lwm2m_resource_t* p_instRes = NULL;
  void* p_resData;

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );
  EMB6_ASSERT_RET( _status == e_lwm2m_api_status_stopped, -1 );

  /* get the object ID, instance ID and resource ID, type and variable length*/
  EMB6_ASSERT_RET( cmdLen >= sizeof(uint16_t), -2 );
  LWM2M_API_GET_FIELD( objId, p_data, cmdLen, uint16_t );
  EMB6_ASSERT_RET( cmdLen >= sizeof(uint8_t), -2 );
  LWM2M_API_GET_FIELD( instId, p_data, cmdLen, uint8_t );
  EMB6_ASSERT_RET( cmdLen >= sizeof(uint16_t), -2 );
  LWM2M_API_GET_FIELD( resId, p_data, cmdLen, uint16_t );
  EMB6_ASSERT_RET( cmdLen >= sizeof(uint8_t), -2 );
  LWM2M_API_GET_FIELD( type, p_data, cmdLen, uint8_t );
  EMB6_ASSERT_RET( cmdLen >= sizeof(uint8_t), -2 );
  LWM2M_API_GET_FIELD( varLen, p_data, cmdLen, uint8_t );
  EMB6_ASSERT_RET( cmdLen >= sizeof(uint8_t), -2 );
  LWM2M_API_GET_FIELD( access, p_data, cmdLen, uint8_t );

  objId = uip_ntohs( objId );
  resId = uip_ntohs( resId );

  /* try to get the according object */
  for( i = 0; i < LWM2MAPI_OBJ_MAX; i++ )
  {
    if((lwm2m_object_container[i] != NULL) &&
        (lwm2m_object_container[i]->id == objId) )
    {
      p_lwm2mObj = lwm2m_object_container[i];
      break;
    }
  }

  if( p_lwm2mObj != NULL )
  {
    /* object already exists ... check for the instance */
    p_lwm2mInst = p_lwm2mObj->p_instances;
    for( i = 0; (i < p_lwm2mObj->count) && (p_lwm2mInst != NULL); i++ )
    {
      if( p_lwm2mInst->id == instId )
        /* we found the according instance ID */
        break;
      p_lwm2mInst = p_lwm2mInst->p_next;
    }

    if( (i >= p_lwm2mObj->count) || (p_lwm2mInst == NULL) )
      /* instance was not found */
      p_lwm2mInst = NULL;
  }


  if( p_lwm2mInst != NULL )
  {
    ret = -1;
    /* allocate memory for the resources */
    p_res = memb_allocm( &lwm2mresource_storage, 1 );
    p_instRes = p_lwm2mInst->p_resources;

    if( p_res != NULL )
    {
      switch( type )
      {
        case e_lwm2m_api_restype_bool:

          varLen = sizeof(int);
          /* try to allocate data storage for the resource data */
          p_resData = memb_allocm( &lwm2mdata_storage, varLen );
          if( p_resData != NULL )
          {
            memset( p_resData, 0, varLen );
            *p_res = (lwm2m_resource_t) {resId, LWM2M_RESOURCE_TYPE_BOOLEAN_VARIABLE,
                    .value.booleanvar.var = p_resData,
                    .p_next = NULL};

            if( (cmdLen > 0) && (cmdLen < varLen) )
              ret = -2;
            else
              ret = 0;

            if( (ret == 0) && (cmdLen > 0) )
            {
              int val;
              /* get the value */
              LWM2M_API_GET_FIELD( val, p_data, cmdLen, int );
              val = uip_ntohl( val );
              memcpy( p_resData, &val, sizeof(int) );
            }
          }
          break;

        case e_lwm2m_api_restype_int:

          varLen = sizeof(int32_t);
          /* try to allocate data storage for the resource data */
          p_resData = memb_allocm( &lwm2mdata_storage, varLen );
          if( p_resData != NULL )
          {
            memset( p_resData, 0, varLen );
            *p_res = (lwm2m_resource_t) {resId, LWM2M_RESOURCE_TYPE_INT_VARIABLE,
                    .value.integervar.var = p_resData,
                    .p_next = NULL};

            if( (cmdLen > 0) && (cmdLen < varLen) )
              ret = -2;
            else
              ret = 0;

            if( (ret == 0) && (cmdLen > 0) )
            {
              int32_t val;
              /* get the value */
              LWM2M_API_GET_FIELD( val, p_data, cmdLen, int32_t );
              val = uip_ntohl( val );
              memcpy( p_resData, &val, sizeof(int32_t) );
            }

          }
          break;

        case e_lwm2m_api_restype_float:

          varLen = sizeof(float);
          /* try to allocate data storage for the resource data */
          p_resData = memb_allocm( &lwm2mdata_storage, varLen );
          if( p_resData != NULL )
          {
            memset( p_resData, 0, varLen );
            *p_res = (lwm2m_resource_t) {resId, LWM2M_RESOURCE_TYPE_FLOATFIX_VARIABLE,
                    .value.floatfixvar.var = p_resData,
                    .p_next = NULL};

            if( (cmdLen > 0) && (cmdLen < varLen) )
              ret = -2;
            else
              ret = 0;

            if( (ret == 0) && (cmdLen > 0) )
            {
              int32_t val;
              /* get the value */
              LWM2M_API_GET_FIELD( val, p_data, cmdLen, int32_t );
              val = uip_ntohl( val );
              memcpy( p_resData, &val, sizeof(int32_t) );
            }
          }
          break;

        case e_lwm2m_api_restype_str:

          /* try to allocate data storage for the resource data */
          p_resData = memb_allocm( &lwm2mdata_storage, varLen +
              sizeof(uint8_t*) + sizeof(uint16_t) );
          if( p_resData != NULL )
          {
            memset( p_resData, 0, varLen );

            *((uint8_t**)p_resData) =  (uint8_t*)((uint8_t*)p_resData + sizeof(uint16_t) + sizeof(uint8_t*));
            *p_res = (lwm2m_resource_t) {resId, LWM2M_RESOURCE_TYPE_STR_VARIABLE,
                    .value.stringvar.size = varLen,
                    .value.stringvar.len = (uint16_t*)((uint8_t*)p_resData + sizeof(uint8_t*)),
                    .value.stringvar.var = p_resData,
                    .p_next = NULL};

            if( (cmdLen > 0) && (cmdLen > p_res->value.stringvar.size) )
              ret = -2;
            else
              ret = 0;

            if( (ret == 0) && (cmdLen > 0) )
            {
              memset( *p_res->value.stringvar.var, 0,
                  p_res->value.stringvar.size);
              *p_res->value.stringvar.len = cmdLen;
              LWM2M_API_GET_FIELD_MEM( *p_res->value.stringvar.var,
                  p_data, cmdLen, cmdLen );
            }
          }
          break;


        default:
          ret = -1;
      }
    }
  }

  if( ret == 0 )
  {
    while( (p_instRes != NULL) && (p_instRes->p_next != NULL) )
      p_instRes = p_instRes->p_next;

    if( p_instRes != NULL )
      p_instRes->p_next = p_res;
    else
      p_lwm2mInst->p_resources = p_res;
    p_lwm2mInst->count++;

    ret = 0;
  }

  /* set type of the response frame */
  EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_frameID_t), -1 );
  LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_frameID_t,
     e_lwm2m_api_type_res_ret);

  /* set Object ID of the response frame */
  EMB6_ASSERT_RET( rplLen >= sizeof(uint16_t), -1 );
  LWM2M_API_SET_FIELD( p_txBuf, rplLen, uint16_t, uip_ntohs( objId ));
  /* set instance ID of the response frame */
  EMB6_ASSERT_RET( rplLen >= sizeof(uint8_t), -1 );
  LWM2M_API_SET_FIELD( p_txBuf, rplLen, uint8_t,  instId );
  /* set Resource ID of the response frame */
  EMB6_ASSERT_RET( rplLen >= sizeof(uint16_t), -1 );
  LWM2M_API_SET_FIELD( p_txBuf, rplLen, uint16_t, uip_ntohs( resId ));

  if( ret == 0 )
  {
    /* set status OK */
    EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_ret_t), -1 );
    LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_ret_t, e_lwm2m_api_ret_ok );
  }
  else
  {
    if( p_res != NULL )
      memb_free( &lwm2mresource_storage, p_res );
    if( p_resData != NULL )
      memb_free( &lwm2mdata_storage, p_resData );

    /* set status OK */
    EMB6_ASSERT_RET( rplLen >= sizeof(lwm2mapi_ret_t), -1 );
    LWM2M_API_SET_FIELD( p_txBuf, rplLen, lwm2mapi_ret_t, e_lwm2m_api_ret_error );
  }

  ret = p_txBuf - p_rpl;
  return ret;
}


/**
 * \brief   Callback to delete a LWM2M resource
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_res_delete( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
#if 0
  int32_t ret = 0;
  uint8_t* p_data = p_cmd;
  uint8_t* p_txBuf = p_rpl;
  int i;
#endif

  EMB6_ASSERT_RET( p_cmd != NULL, 0 );
  EMB6_ASSERT_RET( p_rpl != NULL, 0 );
  EMB6_ASSERT_RET( _status == e_lwm2m_api_status_stopped, -1 );

  return -1;
}

#endif /* #if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE */


/**
 * \brief   Callback to write a LWM2M resource.
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_res_wr( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret = 0;
  uint8_t* p_data = p_cmd;

  uint16_t objId;
  uint8_t instId;
  uint16_t resId;

  const lwm2m_object_t* p_lwm2mObj;
  lwm2m_instance_t* p_lwm2mInst;
  const lwm2m_resource_t* p_lwm2mRes;


  char* p_url;
  int urlLen = 0;

  EMB6_ASSERT_RET( p_cmd != NULL, -1 );
  EMB6_ASSERT_RET( p_rpl != NULL, -1 );

  if( ret == 0 )
  {
    /* get the object ID, instance ID and resource ID*/
    EMB6_ASSERT_RET( cmdLen >= sizeof(uint16_t), -2 );
    LWM2M_API_GET_FIELD( objId, p_data, cmdLen, uint16_t );
    EMB6_ASSERT_RET( cmdLen >= sizeof(uint8_t), -2 );
    LWM2M_API_GET_FIELD( instId, p_data, cmdLen, uint8_t );
    EMB6_ASSERT_RET( cmdLen >= sizeof(uint16_t), -2 );
    LWM2M_API_GET_FIELD( resId, p_data, cmdLen, uint16_t );

    objId = uip_ntohs( objId );
    resId = uip_ntohs( resId );

    /* try to get the according object */
    p_lwm2mObj = lwm2m_engine_get_object( objId );
    if( p_lwm2mObj == NULL )
      ret = -1;
  }

  if( ret == 0 )
  {
    p_lwm2mInst = p_lwm2mObj->p_instances;
    int i = 0;
    for( i = 0; (i < p_lwm2mObj->count) && (p_lwm2mInst != NULL); i++ )
    {
      if( p_lwm2mInst->id == instId )
        /* we found the according instance ID */
        break;
      p_lwm2mInst = p_lwm2mInst->p_next;
    }

    if( i >= p_lwm2mObj->count )
      /* instance was not found */
      ret = -1;
  }

  if( ret == 0 )
  {
    p_lwm2mRes = p_lwm2mInst->p_resources;
    int i = 0;
    for( i = 0; (i < p_lwm2mInst->count) && (p_lwm2mRes != NULL); i++ )
    {
      if( p_lwm2mRes->id == resId )
        /* we found the according resource ID */
        break;
      p_lwm2mRes = p_lwm2mRes->p_next;
    }

    if( i >= p_lwm2mInst->count )
      /* resource was not found */
      ret = -1;
  }

  if( ret == 0 )
  {
    /* get the URL */
    urlLen = _get_lwm2m_url( &p_lwm2mInst->id, &p_lwm2mRes->id, NULL,
        &p_url );
    if( urlLen == 0)
      ret = -1;
  }

  if( ret == 0 )
  {
    /* write the resource value */
    ret = _wr_res( p_lwm2mRes, &p_data, &cmdLen, NULL );
  }

  if( ret == 0 )
  {

    if( _status == e_lwm2m_api_status_registered )
    {
      /* Notify all observers */
      lwm2m_object_notify_observers( p_lwm2mObj, p_url );

    }

    EMB6_ASSERT_RET( p_rpl != NULL, -1 );
    ret = _rsp_status( p_rpl, rplLen, e_lwm2m_api_type_ret,
        e_lwm2m_api_ret_ok );
  }

  return ret;
}


/**
 * \brief   Callback to write a LWM2M instance.
 *
 * \param   p_cmd   Further data of the command.
 * \param   cmdLen  Length of the command data.
 * \param   p_rpl   Pointer to store the response to.
 * \param   rplLen  Length of the response buffer.
 *
 * \return  The length of the generated response or 0 if no response
 *          has been generated.
 */
static int32_t _hndl_inst_wr( uint8_t* p_cmd, uint16_t cmdLen,
    uint8_t* p_rpl, uint16_t rplLen )
{
  int32_t ret = 0;
  uint8_t* p_data = p_cmd;

  uint16_t objId;
  uint8_t instId;
  uint8_t resNum;
  uint16_t resId;
  uint8_t resSize;

  const lwm2m_object_t* p_lwm2mObj;
  lwm2m_instance_t* p_lwm2mInst;
  const lwm2m_resource_t* p_lwm2mRes;


  char* p_url;
  int urlLen = 0;

  EMB6_ASSERT_RET( p_cmd != NULL, -1 );
  EMB6_ASSERT_RET( p_rpl != NULL, -1 );

  if( ret == 0 )
  {
    /* get the object ID, instance ID and resource ID*/
    EMB6_ASSERT_RET( cmdLen >= sizeof(uint16_t), -2 );
    LWM2M_API_GET_FIELD( objId, p_data, cmdLen, uint16_t );
    EMB6_ASSERT_RET( cmdLen >= sizeof(uint8_t), -2 );
    LWM2M_API_GET_FIELD( instId, p_data, cmdLen, uint8_t );
    EMB6_ASSERT_RET( cmdLen >= sizeof(uint16_t), -2 );
    LWM2M_API_GET_FIELD( resNum, p_data, cmdLen, uint8_t );

    objId = uip_ntohs( objId );

    /* try to get the according object */
    p_lwm2mObj = lwm2m_engine_get_object( objId );
    if( p_lwm2mObj == NULL )
      ret = -1;
  }

  if( ret == 0 )
  {
    p_lwm2mInst = p_lwm2mObj->p_instances;
    int i = 0;
    for( i = 0; (i < p_lwm2mObj->count) && (p_lwm2mInst != NULL); i++ )
    {
      if( p_lwm2mInst->id == instId )
        /* we found the according instance ID */
        break;
      p_lwm2mInst = p_lwm2mInst->p_next;
    }

    if( i >= p_lwm2mObj->count )
      /* instance was not found */
      ret = -1;
  }

  if( ret == 0 )
  {
    /* get the URL */
    urlLen = _get_lwm2m_url( &p_lwm2mInst->id, NULL, NULL,
        &p_url );
    if( urlLen == 0)
      ret = -1;
  }

  if( ret == 0 )
  {
    int i = 0;
    for( i = 0; i < resNum; i++ )
    {
      EMB6_ASSERT_RET( cmdLen >= sizeof(uint16_t), -2 );
      LWM2M_API_GET_FIELD( resId, p_data, cmdLen, uint16_t );
      LWM2M_API_GET_FIELD( resSize, p_data, cmdLen, uint8_t );
      resId = uip_ntohs( resId );


      p_lwm2mRes = p_lwm2mInst->p_resources;
      int j = 0;
      for( j = 0; (j < p_lwm2mInst->count) && (p_lwm2mRes != NULL); j++ )
      {
        if( p_lwm2mRes->id == resId )
          /* we found the according resource ID */
          break;
        p_lwm2mRes = p_lwm2mRes->p_next;
      }

      if( j >= p_lwm2mInst->count )
      {
        /* resource was not found */
        ret = -1;
        break;
      }
      else
      {
        /* Resource was found. Write the resource value */
        ret = _wr_res( p_lwm2mRes, &p_data, &cmdLen, &resSize );
        if(ret != 0)
            break;
      }
    }
  }

  if( ret == 0 )
  {

    if( _status == e_lwm2m_api_status_registered )
    {
      /* Notify all observers */
      lwm2m_object_notify_observers( p_lwm2mObj, p_url );

    }

    EMB6_ASSERT_RET( p_rpl != NULL, -1 );
    ret = _rsp_status( p_rpl, rplLen, e_lwm2m_api_type_ret,
        e_lwm2m_api_ret_ok );
  }

  return ret;
}


static int32_t _wr_res( const lwm2m_resource_t* p_lwm2mRes, uint8_t** p_cmd,
    uint16_t* p_cmdLen, uint8_t* varLen )
{
  int32_t ret = 0;
  uint8_t typeA;
  uint8_t typeB;


  if( p_lwm2mRes->type == LWM2M_RESOURCE_TYPE_CALLBACK )
  {
    typeA = p_lwm2mRes->subtype;
    typeB = p_lwm2mRes->type;
  }
  else
  {
    typeA = p_lwm2mRes->type;
    typeB = p_lwm2mRes->subtype;
  }

  /* resource was found. Now we can read the value from it */
  switch( typeA )
  {
    int len;
    /* String Write */
    case LWM2M_RESOURCE_TYPE_STR_VARIABLE:

      EMB6_ASSERT_RET( *p_cmdLen > 0, -2 );
      if( varLen == NULL )
      {
        EMB6_ASSERT_RET( *p_cmdLen < p_lwm2mRes->value.stringvar.size, -2 );
        len = *p_cmdLen;
      }
      else
      {
        EMB6_ASSERT_RET( *varLen < p_lwm2mRes->value.stringvar.size, -2 );
        len = *varLen;
      }

      if( typeB == LWM2M_RESOURCE_TYPE_CALLBACK )
      {
        EMB6_ASSERT_RET( p_lwm2mRes->value.callback.set != NULL, -1 );
        if( p_lwm2mRes->value.callback.set( *p_cmd, len ) < 0 )
          ret = -3;
      }
      else
      {
        EMB6_ASSERT_RET( len < p_lwm2mRes->value.stringvar.size, -3 );
        memset( *p_lwm2mRes->value.stringvar.var, 0,
            p_lwm2mRes->value.stringvar.size);
        *p_lwm2mRes->value.stringvar.len = len;
        LWM2M_API_GET_FIELD_MEM( *p_lwm2mRes->value.stringvar.var,
            *p_cmd, *p_cmdLen, len );
      }
      break;

    /* Integer write */
    case LWM2M_RESOURCE_TYPE_INT_VARIABLE:
    {
      int32_t val;
      if( *p_cmdLen < sizeof(val) )
        ret = -2;

      /* get the value */
      EMB6_ASSERT_RET( *p_cmdLen >= sizeof(val), -2 );
      LWM2M_API_GET_FIELD( val, *p_cmd, *p_cmdLen, int32_t );
      val = uip_ntohl( val );

      if( ret == 0 )
      {
        if( typeB == LWM2M_RESOURCE_TYPE_CALLBACK )
        {
          EMB6_ASSERT_RET( p_lwm2mRes->value.callback.set != NULL, -1 );
          if( p_lwm2mRes->value.callback.set( &val, sizeof(val) ) < 0 )
            ret = -3;
        }
        else
        {
          /* write the value*/
          *p_lwm2mRes->value.integervar.var = val;
        }
      }
      break;
    }

    /* Float write */
    case LWM2M_RESOURCE_TYPE_FLOATFIX_VARIABLE:
    {
      union {
        float        f;
        unsigned int i;
      } val;

      if( *p_cmdLen < sizeof(val) )
        ret = -2;

      /* get the value */
      EMB6_ASSERT_RET( *p_cmdLen >= sizeof(val), -2 );
      LWM2M_API_GET_FIELD_MEM( &val, *p_cmd, *p_cmdLen, sizeof(val) );
      val.i = uip_ntohl( val.i );
      val.i = val.f * LWM2M_FLOAT32_FRAC;

      if( ret == 0 )
      {
        if( typeB == LWM2M_RESOURCE_TYPE_CALLBACK )
        {
          EMB6_ASSERT_RET( p_lwm2mRes->value.callback.set != NULL, -1 );
          if( p_lwm2mRes->value.callback.set( &val, sizeof(val) ) < 0 )
            ret = -3;
        }
        else
        {
          /* write the value*/
          *p_lwm2mRes->value.floatfixvar.var = val.i;
        }
      }
      break;
    }

    /* Boolean write */
    case LWM2M_RESOURCE_TYPE_BOOLEAN_VARIABLE:
    {
      int32_t val;
      if( *p_cmdLen < sizeof(int32_t) )
        ret = -2;

      /* get the value */
      EMB6_ASSERT_RET( *p_cmdLen >= sizeof(val), -2 );
      LWM2M_API_GET_FIELD( val, *p_cmd, *p_cmdLen, int32_t );
      val = uip_ntohl( val );

      if( ret == 0 )
      {
        if( typeB == LWM2M_RESOURCE_TYPE_CALLBACK )
        {
          EMB6_ASSERT_RET( p_lwm2mRes->value.callback.set != NULL, -1 );
          if( p_lwm2mRes->value.callback.set( &val, sizeof(val) ) < 0 )
            ret = -3;
        }
        else
        {
          /* write the value*/
          val = val > 0 ? 1 : 0;
          *p_lwm2mRes->value.booleanvar.var = val;
        }
      }
      break;
    }

    default:
      ret = -1;
      break;
  }

  return ret;
}

#if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE
/**
 * \brief   Generic LWM2M handler to GET a resource.
 *
 *          This function is called from the underlying CoAP engine
 *          if a GET access to a resource was requested. The target
 *          of the request can be seen from the parameters.
 *          The request will be forwarded to the LWM2M engine.
 *
 */
static void lwm2m_get(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user)
{
  /* forward request */
  lwm2m_engine_handler((lwm2m_object_t*)p_user, request, response, buffer,
      preferred_size, offset);
}


/**
 * \brief   Generic LWM2M handler to PUT a resource.
 *
 *          This function is called from the underlying CoAP engine
 *          if a PUT access to a resource was requested. The target
 *          of the request can be seen from the parameters.
 *          The request will be forwarded to the LWM2M engine.
 *
 */
static void lwm2m_put(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user)
{
  /* forward request */
  lwm2m_engine_handler((lwm2m_object_t*)p_user, request, response, buffer,
      preferred_size, offset);
}


/**
 * \brief   Generic LWM2M handler to POST a resource.
 *
 *          This function is called from the underlying CoAP engine
 *          if a POST access to a resource was requested. The target
 *          of the request can be seen from the parameters.
 *          The request will be forwarded to the LWM2M engine.
 *
 */
static void lwm2m_post(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user)
{
  /* forward request */
  lwm2m_engine_handler((lwm2m_object_t*)p_user, request, response, buffer,
      preferred_size, offset);
}


/**
 * \brief   Generic LWM2M handler to DELETE a resource.
 *
 *          This function is called from the underlying CoAP engine
 *          if a DELETE access to a resource was requested. The target
 *          of the request can be seen from the parameters.
 *          The request will be forwarded to the LWM2M engine.
 *
 */
static void lwm2m_delete(void *request, void *response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset, void* p_user)
{
  /* forward request */
  lwm2m_engine_delete_handler((lwm2m_object_t*)p_user, request, response, buffer,
      preferred_size, offset);
}
#endif /* #if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE */

/*
 *  --- Global Functions  ---------------------------------------------------*
 */
int8_t lwm2mApiRegister( fn_lwm2m_ApiInit_t pf_init ) {

  EMB6_ASSERT_RET( (pf_init != NULL), -1 );

  fn_lwm2mModule = pf_init;

  return 0;
}
/*---------------------------------------------------------------------------*/
/*
* lwm2mApiInit()
*/
int8_t lwm2mApiInit( uint8_t* p_txBuf, uint16_t txBufLen,
        void(*fn_tx)(uint16_t len, void* p_param), void* p_txParam )
{
    int i;
    int8_t ret = 0;

#if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE
    /* initialize memory */
    memb_init( &lwm2mrestres_storage );
    memb_init( &lwm2mobject_storage );
    memb_init( &lwm2minstance_storage );
    memb_init( &lwm2mresource_storage );
    memb_init( &lwm2mdata_storage );

    /* reset object container */
    for( i = 0; i < LWM2MAPI_OBJ_MAX; i++ )
      lwm2m_object_container[i] = NULL;
#endif /* #if LWM2M_SERIAL_API_SUPPORT_DYN_OBJ == TRUE */


    _serverIP = (uip_ipaddr_t){.u16 = {LWM2MAPI_SERVER_IP}};
    _serverPort = LWM2MAPI_SERVER_PORT;
    snprintf( _epName, sizeof(_epName), LWM2MAPI_ENDPOINT );

    /* set buffer description */
    _p_txBuf = p_txBuf;
    _txBufLen = txBufLen;
    _p_txParam = p_txParam;

    /* set Tx function */
    _fn_tx = fn_tx;

    /* register events */
    evproc_regCallback( EVENT_TYPE_STATUS_CHANGE, _event_callback );
    _status = e_lwm2m_api_status_stopped;

    return ret;
} /* lwm2mApiInit() */


/*---------------------------------------------------------------------------*/
/*
* lwm2mApiInput()
*/
int8_t lwm2mApiInput( uint8_t* p_data, uint16_t len, uint8_t valid )
{
    int ret = -1;

    if( valid )
    {
        /* process the frame */
        ret = _rx_data( p_data, len );
    }

    return ret;
} /* lwm2mApiInput() */

/*---------------------------------------------------------------------------*/
/*
* lwm2mApi_startLWM2M()
*/
int8_t lwm2mApi_startLWM2M() {
  return _startLWM2M();
}

/*---------------------------------------------------------------------------*/
/*
* lwm2mApi_startLWM2M()
*/
int8_t lwm2mApi_stopLWM2M() {
  return _stopLWM2M();
}


