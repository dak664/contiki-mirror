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
 *         Header file for the control of the M25P16 on sensinode N740s.
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#ifndef M25P16_H_
#define M25P16_H_

/* Instruction Set */
#define M25P16_I_WREN      0x06 /* Write Enable */
#define M25P16_I_WRDI      0x04 /* Write Disable */
#define M25P16_I_RDID      0x9F /* Read Identification */
#define M25P16_I_RDSR      0x05 /* Read Status Register */
#define M25P16_I_WRSR      0x01 /* Write Status Register */
#define M25P16_I_READ      0x03 /* Read Data Bytes */
#define M25P16_I_FAST_READ 0x0B /* Read Data Bytes at Higher Speed */
#define M25P16_I_PP        0x02 /* Page Program */
#define M25P16_I_SE        0xD8 /* Sector Erase */
#define M25P16_I_BE        0xC7 /* Bulk Erase */
#define M25P16_I_DP        0xB9 /* Deep Power-down */
#define M25P16_I_RES       0xAB /* Release from Deep Power-down */

/* Dummy Byte - Used in FAST_READ and RES */
#define M25P16_DUMMY_BYTE  0x00

/* Pins */
#define M25P16_PIN_CLOCK   P1_5
#define M25P16_PIN_SER_I   P1_6
#define M25P16_PIN_SER_O   P1_7

/* Status Register Bits */
#define M25P16_SR_SRWD     0x80 /* Status Register Write Disable */
#define M25P16_SR_BP2      0x10 /* Block Protect 2 */
#define M25P16_SR_BP1      0x08 /* Block Protect 1 */
#define M25P16_SR_BP0      0x04 /* Block Protect 0 */
#define M25P16_SR_WEL      0x02 /* Write Enable Latch */
#define M25P16_SR_WIP      0x01 /* Write in Progress */

/* Device Identifier */
struct m25p16_rdid {
  uint8_t  man_id;
  uint8_t  mem_type;
  uint8_t  mem_size;
  uint8_t  uid_len;
  uint8_t  uid[16];
};

/* Check for Write in Progress (1: In progress) */
#define M25P16_WIP() (m25p16_rdsr() & M25P16_SR_WIP)
/* Check for Write Enable (1: Enabled) */
#define M25P16_WEL() (m25p16_rdsr() & M25P16_SR_WEL)

/* Functions */
void m25p16_wren();                            /* Write Enable Instruction */
void m25p16_wrdi();                            /* Write Disable Instruction */
void m25p16_rdid(struct m25p16_rdid * rdid);   /* Read Identification */
uint8_t m25p16_rdsr();                         /* Read Status Register */
void m25p16_wrsr(uint8_t val);                 /* Write Status Reg. */
/* Read and Fast Read from Flash */
void m25p16_read(uint8_t * addr, uint8_t * buff, uint8_t buff_len);
void m25p16_read_fast(uint8_t * addr, uint8_t * buff, uint8_t buff_len);
/* Program Page */
void m25p16_pp(uint8_t * addr, uint8_t * buff, uint8_t buff_len);
void m25p16_se(uint8_t s); /* Sector Erase */
void m25p16_be();          /* Bulk Erase */
void m25p16_dp();          /* Deep Power down */
void m25p16_res();         /* Release from Deep Power Down */
uint8_t m25p16_res_res();  /* Release from Deep Power Down and Read ES */

#endif /* M25P16_H_ */
