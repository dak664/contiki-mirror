/*
 * Copyright (c) 2010, Loughborough University - Computer Science
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
 *
 *         Tool to program the external flash on the N740s via USB
 *
 *         By naming this file serial-line.c, it overrides the file with the
 *         same name in core.
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */
#include "dev/serial-line.h"
#include "dev/dma.h"
#include "dev/leds.h"
#include "dev/m25p16.h"
#include "sys/ctimer.h"
#include "dev/n740.h"

#include "sensinode-debug.h"
#include <stdio.h>

/* Max interval in secs between commands */
#define RESET_TIMEOUT (10 * CLOCK_SECOND)
#define FORMAT_TIMEOUT (45 * CLOCK_SECOND)

/* Commands */
#define COMMAND_READ 0x0B /* Read Data Bytes at Higher Speed */
#define COMMAND_PP   0x02 /* Page Program */
#define COMMAND_SE   0xD8 /* Sector Erase */
#define COMMAND_BE   0xC7 /* Bulk Erase */
#define COMMAND_NULL 0x00 /* Dummy Command*/
#define COMMAND_WIP  0x05 /* Write in Progress */

#define ERR_GENERIC     0xFF /* Generic Error */
#define ERR_BAD_LEN     0xFE /* Incorrect Length */
#define ERR_NO_INIT     0xFD /* Not Initialised */
#define ERR_BAD_OFFSET  0xFC /* Bad Offset */
#define ERR_PROTECTED   0xFB /* Protected Memory Area */
#define ERR_BAD_MAGIC   0xFA /* Bad Magic Number */

/* Buffer for RX Data */
#define MAX_DATA_LEN 132
#define HEADER_LEN     6
struct pdu {
  uint8_t len; /* Length, including this field. Must stay 1st in the struct */
  uint8_t magic[4];
  uint8_t cmd;
  uint8_t data[MAX_DATA_LEN];
};
static struct pdu cmd;

#define FLASH_DATA_ADDR_LEN   3
#define FLASH_DATA_DATA_LEN 128
struct flash_data {
  uint8_t address[FLASH_DATA_ADDR_LEN];
  uint8_t data[FLASH_DATA_DATA_LEN];
};
static struct flash_data * data_ptr;

#define DMA_CHANNEL 1 /* Use DMA Channel for UART1 RX */
/* Structure to hold the DMA channel configuration */
static struct dma_config * dma_descriptor;
extern struct process * dma_callback[4];

static uint8_t wip; /* Work in Progress - read or write */
static struct ctimer c;
static const uint8_t magic[] = { 0xB0, 0xC1, 0xD8, 0xFE };
/*---------------------------------------------------------------------------*/
PROCESS(serial_line_process, "Serial driver");
/*---------------------------------------------------------------------------*/
static void
sequence_reset()
{
  wip = 0;
  ctimer_stop(&c);
  m25p16_dp();
}
/*---------------------------------------------------------------------------*/
static void
ctimer_callback(void * p)
{
  n740_analog_deactivate();
  sequence_reset();
  n740_analog_activate();
}
/*---------------------------------------------------------------------------*/
static void
configure_dma()
{
  dma_descriptor = &dma_conf[DMA_CHANNEL];

  /* Configure the DMA channel */
  dma_descriptor->src_h = 0xDF; /* SFR access via XDATA - offset */
  dma_descriptor->src_l = 0xF9; /* SFR access via XDATA - U1BUF */
  dma_descriptor->dst_h = ((uint16_t) &cmd) >> 8;
  dma_descriptor->dst_l = ((uint16_t) &cmd);
  dma_descriptor->len_h = DMA_VLEN_N;
  dma_descriptor->len_l = MAX_DATA_LEN + 6;
  /* Byte 6: Wordsize=0, Mode=Single, Trigger=UART1 RX */
  dma_descriptor->wtt = DMA_T_URX1;
  /* Byte 7: src no incr, dst incr, M8=0, Prio=low, IRQ=on */
  dma_descriptor->inc_prio = DMA_DST_INC_1 | DMA_IRQ_MASK_ENABLE;

  dma_associate_process(&serial_line_process, DMA_CHANNEL);
}
/*---------------------------------------------------------------------------*/
static void
send_response() {
  uint8_t i;
  uint8_t * ptr;
  ptr = (uint8_t *) &cmd;
  for(i = 0; i < cmd.len; i++) {
    uart1_writeb(ptr[i]);
  }
}
/*---------------------------------------------------------------------------*/
static void
input_data()
{
  n740_analog_deactivate();

  /* Ignore PDUs with incorrect magic */
  if(memcmp(cmd.magic, magic, 4) != 0) {
    cmd.len = HEADER_LEN;
    cmd.cmd = ERR_BAD_MAGIC;
    memcpy(cmd.magic, magic, 4);
  }

  /* Always accept Init */
  if(cmd.len == HEADER_LEN && cmd.cmd == COMMAND_NULL) {
    wip = 1;
    ctimer_set(&c, RESET_TIMEOUT, ctimer_callback, NULL);
    m25p16_res();
  }

  if((wip == 1) && (cmd.cmd != COMMAND_NULL)) {
    /* Execute the Command */
    ctimer_set(&c, RESET_TIMEOUT, ctimer_callback, NULL);

    switch (cmd.cmd) {
    case M25P16_I_RDSR:
      cmd.len = HEADER_LEN + 1;
      cmd.data[0] = M25P16_WIP();
      break;
    case M25P16_I_FAST_READ:
      if (cmd.len != (HEADER_LEN + FLASH_DATA_ADDR_LEN)) {
        cmd.len = HEADER_LEN;
        cmd.cmd = ERR_BAD_LEN;
        break;
      }
      if (data_ptr->address[0] > 0x1F) {
        cmd.len = HEADER_LEN;
        cmd.cmd = ERR_BAD_OFFSET;
        break;
      }
      m25p16_read_fast(data_ptr->address, data_ptr->data, 128);
      cmd.len = HEADER_LEN + FLASH_DATA_ADDR_LEN + FLASH_DATA_DATA_LEN;
      break;
    case M25P16_I_SE:
      if (cmd.len != (HEADER_LEN + 1)) {
        cmd.len = HEADER_LEN;
        cmd.cmd = ERR_BAD_LEN;
        break;
      }
      if (data_ptr->address[0] > 0x1F) {
        cmd.len = HEADER_LEN;
        cmd.cmd = ERR_BAD_OFFSET;
        break;
      }
      m25p16_se(cmd.data[0]);
      break;
    case M25P16_I_PP:
      if (cmd.len != (HEADER_LEN + FLASH_DATA_ADDR_LEN + FLASH_DATA_DATA_LEN)) {
        cmd.len = HEADER_LEN;
        cmd.cmd = ERR_BAD_LEN;
        break;
      }
      if (data_ptr->address[0] > 0x1F) {
        cmd.len = HEADER_LEN;
        cmd.cmd = ERR_BAD_OFFSET;
        break;
      }
      m25p16_pp(data_ptr->address, data_ptr->data, 128);
      cmd.len = HEADER_LEN;
      while(M25P16_WIP());
      break;
    case M25P16_I_BE:
      if (cmd.len != HEADER_LEN) {
        cmd.len = HEADER_LEN;
        cmd.cmd = ERR_BAD_LEN;
        break;
      }
      /* Check SRWD and BP[2:0] bits in the SR */
      cmd.data[0] = m25p16_rdsr();

      if((cmd.data[0] & 0x9c) != 0) {
        cmd.len = HEADER_LEN + 1;
        cmd.cmd = ERR_PROTECTED;
        break;
      }
      ctimer_set(&c, FORMAT_TIMEOUT, ctimer_callback, NULL);
      cmd.len = HEADER_LEN;
      cmd.cmd = M25P16_I_BE;
      m25p16_be();
      break;
    }
  } else if((wip == 0) && (cmd.cmd != COMMAND_NULL)) {
    cmd.len = HEADER_LEN;
    cmd.cmd = ERR_NO_INIT;
  }

  /* Some error occured */
  if(cmd.cmd >= 0xF0) {
    sequence_reset();
  }

  if(memcmp(cmd.magic, magic, 4) != 0) {
    cmd.len = HEADER_LEN;
    cmd.cmd = ERR_BAD_MAGIC;
    memcpy(cmd.magic, magic, 4);
  }

  n740_analog_activate();
  send_response();
}
/*---------------------------------------------------------------------------*/
int
serial_line_input_byte(unsigned char c)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_line_process, ev, data)
{

  PROCESS_BEGIN();

  /* Configure and arm a DMA channel */
  configure_dma();
  DMA_ARM(DMA_CHANNEL);

  wip = 0;
  data_ptr = (struct flash_data *)cmd.data;

  while(1) {
    PROCESS_YIELD();
    if(ev == PROCESS_EVENT_POLL) {
      leds_on(LEDS_GREEN);
      input_data();
      DMA_ARM(DMA_CHANNEL);
      leds_off(LEDS_GREEN);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
serial_line_init(void)
{
  process_start(&serial_line_process, NULL);
}
/*---------------------------------------------------------------------------*/
