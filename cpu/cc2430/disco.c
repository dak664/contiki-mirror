/*
 * Copyright (c) 2010, Loughborough University - Computer Science
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
 *         Disco server sources
 *         (embedded part of the DISCOBALL project)
 *
 *         It objective is to receive a code file over UDP, store it in
 *         external flash and disseminate it to other nodes of the
 *         6LoWPAN network.
 *
 *         For this to work, the image must be co-hosted with the BooTTY!
 *         bootloader, which will move the image from external to internal
 *         flash.
 *
 *         To link this application in your contiki image, all you need to
 *         do is to add this line:
 *         OFFSET_FIRMWARE=1
 *         to your project's makefile
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#include "contiki.h"
#include "contiki-net.h"
#include "sys/clock.h"
#include "sys/ctimer.h"
#include "dev/watchdog.h"

#include "dev/n740.h"
#include "dev/m25p16.h"

#include "disco.h"
/*---------------------------------------------------------------------------*/
#define DEBUG DEBUG_NONE
#include "net/uip-debug.h"
/*---------------------------------------------------------------------------*/
static struct uip_udp_conn *server_conn;
static struct disco_request_pdu * req;
static struct disco_response_pdu resp;
static struct disco_seed seed;
static uint8_t state;
static uint8_t sector;
static uint16_t interval;
static struct ctimer disco_timer;

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

extern uint16_t uip_len;
extern void *uip_appdata;

__xdata __at(BOOTTY_CMD_LOCATION) static uint8_t bd;
/*---------------------------------------------------------------------------*/
static void timer_handler(void * p);
/*---------------------------------------------------------------------------*/
static void
abort()
{
  PRINTF("Abort @ %lu\n", clock_seconds());
  n740_analog_deactivate();
  m25p16_dp();
  n740_analog_activate();
  state = DISCO_STATE_LISTENING;
  memset(&seed, 0, sizeof(seed));
  ctimer_stop(&disco_timer);
}
/*---------------------------------------------------------------------------*/
static void
restart_timer(uint16_t t)
{
  interval = t;
  ctimer_stop(&disco_timer);
  ctimer_set(&disco_timer, interval, timer_handler, &interval);
}
/*---------------------------------------------------------------------------*/
static void
timer_handler(void * p)
{
  uint16_t * interval = p;
  uint8_t wip;

  if(*interval == DISCO_TIMEOUT_PREPARE) {
    PRINTF("Prepare @ %lu\n", clock_seconds());

    n740_analog_deactivate();
    wip = M25P16_WIP();
    n740_analog_activate();

    if(wip) {
      restart_timer(DISCO_TIMEOUT_PREPARE);
    } else {
      PRINTF("Erased %u\n", sector);
      if((sector & 1) == 0) {
        sector++;
        PRINTF("Next %u\n", sector);
        n740_analog_deactivate();
        m25p16_se(sector);
        n740_analog_activate();
        restart_timer(DISCO_TIMEOUT_PREPARE);
      } else {
        PRINTF("Ready\n");
        state = DISCO_STATE_READY;
        resp.status = DISCO_CMD_INIT;
        restart_timer(DISCO_TIMEOUT_ABORT);
        server_conn->rport = seed.port;
        uip_ipaddr_copy(&server_conn->ripaddr, &seed.addr);
        uip_udp_packet_send(server_conn, &resp, DISCO_RESP_LEN_INIT);

        /* Restore server connection to allow data from any node */
        uip_create_unspecified(&server_conn->ripaddr);
        server_conn->rport = 0;
      }
    }
  } else if(*interval == DISCO_TIMEOUT_ABORT) {
    abort();
  } else if(*interval == DISCO_TIMEOUT_REBOOT) {
    watchdog_reboot();
  }
}
/*---------------------------------------------------------------------------*/
static uint8_t
cmd_init()
{
  PRINTF("Init 0x%02x\n", req->addr[0]);
  if(uip_datalen() != DISCO_LEN_INIT) {
    PRINTF("Bad len (%u)\n", uip_datalen());
    resp.status = DISCO_ERR_BAD_LEN;
    return DISCO_RESP_LEN_ERR;
  }
  n740_analog_deactivate();
  m25p16_res();
  sector = 2 * req->addr[0];
  m25p16_se(sector);
  n740_analog_activate();
  state = DISCO_STATE_PREPARING;
  restart_timer(DISCO_TIMEOUT_PREPARE);

  /* Store the sender's address/port so we can reply when ready */
  seed.port = UIP_UDP_BUF->srcport;
  uip_ipaddr_copy(&seed.addr, &UIP_IP_BUF->srcipaddr);
  PRINTF("OK\n");
  return DISCO_RESPONSE_NONE;
}
/*---------------------------------------------------------------------------*/
static uint8_t
cmd_write()
{
  PRINTF("Write 0x%02x%02x%02x\n", req->addr[0], req->addr[1], req->addr[2]);
  if(uip_datalen() != DISCO_LEN_WRITE) {
    resp.status = DISCO_ERR_BAD_LEN;
    return DISCO_RESP_LEN_ERR;
  }
  restart_timer(DISCO_TIMEOUT_ABORT);
  n740_analog_deactivate();
  m25p16_pp(req->addr, req->data, DISCO_FLEN_DATA);
  watchdog_periodic();
  while(M25P16_WIP());
  n740_analog_activate();
  resp.status = DISCO_CMD_WRITE;
  memcpy(resp.addr, req->addr, DISCO_FLEN_ADDR);
  return DISCO_RESP_LEN_WRITE;
}
/*---------------------------------------------------------------------------*/
static uint8_t
cmd_switch()
{
  PRINTF("Switch 0x%02x\n", req->addr[0]);
  if(uip_datalen() != DISCO_LEN_SWITCH) {
    resp.status = DISCO_ERR_BAD_LEN;
    return DISCO_RESP_LEN_ERR;
  }
  if(req->addr[0] > 15) {
    resp.status = DISCO_ERR_BAD_OFFSET;
    return DISCO_RESP_LEN_ERR;
  }

  bd = BOOTTY_CMD_COPY_IMAGE;
  bd |= req->addr[0];

  resp.status = DISCO_CMD_SWITCH;
  resp.addr[0] = req->addr[0];

  restart_timer(DISCO_TIMEOUT_REBOOT);

  return DISCO_RESP_LEN_SWITCH;
}
/*---------------------------------------------------------------------------*/
static uint8_t
cmd_done()
{
  PRINTF("Done\n");
  if(uip_datalen() != DISCO_LEN_DONE) {
    resp.status = DISCO_ERR_BAD_LEN;
    return DISCO_RESP_LEN_ERR;
  }
  resp.status = DISCO_CMD_DONE;

  return DISCO_RESP_LEN_DONE;
}
/*---------------------------------------------------------------------------*/
static uint8_t
event_handler(process_event_t ev)
{
  uint8_t rv = DISCO_RESPONSE_NONE;

  if(ev != tcpip_event) {
    return rv;
  }

  /* Always accept CMD_DONE */
  if(req->cmd == DISCO_CMD_DONE) {
    return cmd_done();
  }

  /* Always accept switch too */
  if(req->cmd == DISCO_CMD_SWITCH) {
    return cmd_switch();
  }

  switch(state) {
  case DISCO_STATE_LISTENING:
    req = uip_appdata;
    if(req->cmd == DISCO_CMD_INIT) {
      rv = cmd_init();
    }
    break;
  case DISCO_STATE_PREPARING:
    PRINTF("Not Ready\n");
    resp.status = DISCO_ERR_NOT_READY;
    rv = DISCO_RESP_LEN_ERR;
    break;
  case DISCO_STATE_READY:
    req = uip_appdata;
    if(req->cmd == DISCO_CMD_WRITE) {
      rv = cmd_write();
    } else if(req->cmd == DISCO_CMD_INIT) {
      resp.status = DISCO_ERR_INIT_DONE;
      rv = DISCO_RESP_LEN_ERR;
    } else if(req->cmd == DISCO_CMD_SWITCH) {
      rv = cmd_switch();
    }
    break;
  }
  return rv;
}
/*---------------------------------------------------------------------------*/
PROCESS(disco_process, "Disco Server Process");
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(disco_process, ev, data)
{
  uint8_t len;

  PROCESS_BEGIN();

  PRINTF("Disco Server\n");

  server_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(server_conn, UIP_HTONS(DISCO_UDP_PORT));

  state = DISCO_STATE_LISTENING;

  while(1) {
    PROCESS_YIELD();
    len = event_handler(ev);

    if(len > 0) {
      server_conn->rport = UIP_UDP_BUF->srcport;
      uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
      uip_udp_packet_send(server_conn, &resp, len);
      /* Restore server connection to allow data from any node */
      uip_create_unspecified(&server_conn->ripaddr);
      server_conn->rport = 0;
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/