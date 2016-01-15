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
/*============================================================================*/
/**
 *      \addtogroup emb6
 *      @{
 *      \addtogroup demo_mdns
 *      @{
 *      \addtogroup demo_mdns_server
 *      @{
*/
/*! \file   demo_mdns_srv.c

 \author Fesseha Tsegaye

 \brief  mDNS example server application

 \version 0.0.1
 */
/*============================================================================*/

/*==============================================================================
 INCLUDE FILES
 =============================================================================*/

#include "demo_mdns_srv.h"
#include "emb6.h"

#include "tcpip.h"
#include "udp-socket.h"
#include "uip.h"
#include "resolv.h"


#define     LOGGER_ENABLE        LOGGER_DEMO_MDNS
#include    "logger.h"

/*==============================================================================
                          LOCAL VARIABLE DECLARATIONS
 =============================================================================*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]", (lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3], (lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif

#define MAX_PAYLOAD_LEN 120

static struct udp_socket   st_server_udp_socket;
struct udp_socket*  pst_server_udp_socket;
static int seq_id;

static void mdns_callback(struct udp_socket *c,
		void *ptr,
		const uip_ipaddr_t *source_addr,
		uint16_t source_port,
		const uip_ipaddr_t *dest_addr,
		uint16_t dest_port,
		const uint8_t *data,
		uint16_t datalen)
{
	char buf[MAX_PAYLOAD_LEN];
	memset(buf, 0, MAX_PAYLOAD_LEN);

	if(data) {
		((char *)data)[datalen] = 0;
		PRINTF("Server received: '%s' from ", (char *)data);
		PRINT6ADDR(source_addr);
		PRINTF("\n");

		uip_ipaddr_copy(&pst_server_udp_socket->udp_conn->ripaddr, source_addr);
		PRINTF("Responding with message: ");
		sprintf(buf, "Hello from the server! (%d)", ++seq_id);
		PRINTF("%s\n", buf);

		udp_socket_send(pst_server_udp_socket, buf, strlen(buf));
		/* Restore server connection to allow data from any node */
		memset(&pst_server_udp_socket->udp_conn->ripaddr, 0, sizeof(uip_ipaddr_t));
	}
}
/*==============================================================================
                                         API FUNCTIONS
 =============================================================================*/
/*----------------------------------------------------------------------------*/
/*    demo_mdnsInit()                                                           */
/*----------------------------------------------------------------------------*/

int8_t demo_mdnsInit(void)
{
	init();
#if RESOLV_CONF_SUPPORTS_MDNS
	resolv_set_hostname("contiki-udp-server");
#endif

	pst_server_udp_socket = &st_server_udp_socket;

	udp_socket_register(pst_server_udp_socket, NULL, mdns_callback);
	pst_server_udp_socket->udp_conn->rport = UIP_HTONS(3001);
	udp_socket_bind(pst_server_udp_socket, 3000);

	return 1;
}

/*----------------------------------------------------------------------------*/
/*    demo_coapConf()                                                           */
/*----------------------------------------------------------------------------*/

uint8_t demo_mdnsConf(s_ns_t* pst_netStack)
{
    uint8_t c_ret = 1;

    /*
     * By default stack
     */
    if (pst_netStack != NULL) {
        if (!pst_netStack->c_configured) {
            /*pst_netStack->sock   = &udp_socket_driver;*/
            pst_netStack->hc     = &sicslowpan_driver;
            pst_netStack->llsec  = &nullsec_driver;
            pst_netStack->hmac   = &nullmac_driver;
            pst_netStack->lmac   = &sicslowmac_driver;
            pst_netStack->frame  = &framer_802154;
            pst_netStack->c_configured = 1;
            /* Transceiver interface is defined by @ref board_conf function*/
            /*pst_netStack->inif   = $<some_transceiver>;*/
        } else {
            if (/*(pst_netStack->sock == &udp_socket_driver) && */
                (pst_netStack->hc == &sicslowpan_driver)   &&
                (pst_netStack->llsec == &nullsec_driver)   &&
                (pst_netStack->hmac == &nullmac_driver)    &&
                (pst_netStack->lmac == &sicslowmac_driver) &&
                (pst_netStack->frame == &framer_802154)) {
                /* right configuration */
            }
            else {
                pst_netStack = NULL;
                c_ret = 0;
            }
        }
    }
    return (c_ret);
}

/** @} */
/** @} */
/** @} */
