/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *      An abstraction layer for RESTful Web services (Erbium).
 *      Inspired by RESTful Contiki by Dogan Yazar.
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 *      Peter Lehmann <peter.lehmann@hs-offenburg.de>
 *
 */

#include "emb6.h"
#include "bsp.h"
#include "er-coap.h"
#include "rest-engine.h"
#include "etimer.h"


#define DEBUG DEBUG_NONE
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


/*---------------------------------------------------------------------------*/
  LIST(restful_services);
  LIST(restful_periodic_services);

/* initialize the PERIODIC_RESOURCE timers, which will be handled by this process. */
static periodic_resource_t *periodic_resource = NULL;

void _rest_et_callback(c_event_t c_event, p_data_t p_data);

/*---------------------------------------------------------------------------*/
/*- REST Engine API ---------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * \brief Initializes and starts the REST Engine process
 *
 * This function must be called by server processes before any resources are
 * registered through rest_activate_resource().
 */
void
rest_init_engine(void)
{

  /* initialize list */
  list_init(restful_services);
  REST.set_service_callback(rest_invoke_restful_service);

  /* Start the RESTful server implementation. */
  REST.init();


}
/*---------------------------------------------------------------------------*/
/*- REST Engine API ---------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * \brief Stop the REST Engine process
 *
 * This function must be called by server processes to stop the REST Engine and
 * remove the dedicated UDP connection.
 */
void
rest_engine_stop(void)
{

  /* initialize list */
  list_init(restful_services);

  /* Stop the RESTful server implementation. */
  REST.stop();


}
/*---------------------------------------------------------------------------*/
/**
 * \brief Makes a resource available under the given URI path
 * \param resource A pointer to a resource implementation
 * \param path The URI path string for this resource
 *
 * The resource implementation must be imported first using the
 * extern keyword. The build system takes care of compiling every
 * *.c file in the ./resources/ sub-directory (see example Makefile).
 */
void
rest_activate_resource(resource_t *resource, char *path)
{
  resource->url = path;
  struct periodic_resource_s * periodic = resource->un_handler.periodic;
  list_add(restful_services, resource);

  PRINTF("Activating: %s\n\r", resource->url);

  /* Only add periodic resources with a periodic_handler and a period > 0. */
  if((resource->flags & IS_PERIODIC) && periodic->periodic_handler
     && periodic->period) {
    PRINTF("Periodic resource: %p (%s)\n", periodic,periodic->resource->url);
    list_add(restful_periodic_services, periodic);
  }

  for(periodic_resource =
        (periodic_resource_t *)list_head(restful_periodic_services);
      periodic_resource; periodic_resource = periodic_resource->next) {
    if(periodic_resource->periodic_handler && periodic_resource->period) {
      PRINTF("Periodic: Set timer for /%s to %lu\n\r",
             periodic_resource->resource->url, periodic_resource->period);
      etimer_set(&periodic_resource->periodic_timer,
                 periodic_resource->period * bsp_getTRes(), _rest_et_callback);
    }
  }
}
/*---------------------------------------------------------------------------*/
/*- Internal API ------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
list_t
rest_get_resources(void)
{
  return restful_services;
}
/*---------------------------------------------------------------------------*/
int
rest_invoke_restful_service(void *request, void *response, uint8_t *buffer,
                            uint16_t buffer_size, int32_t *offset)
{
  uint8_t found = 0;
  uint8_t allowed = 1;

  resource_t *resource = NULL;
  const char *url = NULL;
  int url_len, res_url_len;

  url_len = REST.get_url(request, &url);
  for(resource = (resource_t *)list_head(restful_services);
      resource; resource = resource->next) {

    /* if the web service handles that kind of requests and urls matches */
    res_url_len = strlen(resource->url);
    if((url_len == res_url_len
        || (url_len > res_url_len
            && (resource->flags & HAS_SUB_RESOURCES)
            && url[res_url_len] == '/'))
       && strncmp(resource->url, url, res_url_len) == 0) {
      found = 1;
      rest_resource_flags_t method = REST.get_method_type(request);

      PRINTF("/%s, method %u, resource->flags %u\n", resource->url,
             (uint16_t)method, resource->flags);

      if((method & METHOD_GET) && resource->get_handler != NULL) {
        /* call handler function */
        resource->get_handler(request, response, buffer, buffer_size, offset, resource->p_user);
      } else if((method & METHOD_POST) && resource->post_handler != NULL) {
        /* call handler function */
        resource->post_handler(request, response, buffer, buffer_size,
                               offset, resource->p_user);
      } else if((method & METHOD_PUT) && resource->put_handler != NULL) {
        /* call handler function */
        resource->put_handler(request, response, buffer, buffer_size, offset, resource->p_user);
      } else if((method & METHOD_DELETE) && resource->delete_handler != NULL) {
        /* call handler function */
        resource->delete_handler(request, response, buffer, buffer_size,
                                 offset, resource->p_user);
      } else {
        allowed = 0;
        REST.set_response_status(response, REST.status.METHOD_NOT_ALLOWED);
      }
      break;
    }
  }
  if(!found) {
    REST.set_response_status(response, REST.status.NOT_FOUND);
  } else if(allowed) {
    /* final handler for special flags */
    if(resource->flags & IS_OBSERVABLE) {
      REST.subscription_handler(resource, request, response);
    }
  }
  return found & allowed;
}
/*-----------------------------------------------------------------------------------*/
void _rest_et_callback(c_event_t c_event, p_data_t p_data)
{
    if(c_event == EVENT_TYPE_TIMER_EXP) {
      for(periodic_resource =
        (periodic_resource_t *)list_head(restful_periodic_services);
        periodic_resource; 
        periodic_resource = periodic_resource->next) {
          if(periodic_resource->period
            && etimer_expired(&periodic_resource->periodic_timer)) {

            PRINTF("Periodic: etimer expired for /%s (period: %lu)\n",
            periodic_resource->resource->url, periodic_resource->period);

            /* Call the periodic_handler function, which was checked during adding to list. */
            periodic_resource->periodic_handler();
            etimer_restart(&periodic_resource->periodic_timer);
        }
      }
    }
}
/*---------------------------------------------------------------------------*/
