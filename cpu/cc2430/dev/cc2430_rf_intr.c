/**
 * \file
 *         CC2430 RF driver
 * \author
 *         Zach Shelby <zach@sensinode.com>
 *
 *  Non-bankable code for cc2430 rf driver.  
 *  Interrupt routine and code called through function pointers
 *  must be placed into the HOME bank.
 *
 */

#include <stdio.h>

#include "contiki.h"
#include "dev/radio.h"
#include "dev/cc2430_rf.h"
#include "cc2430_sfr.h"
#ifdef RF_LED_ENABLE
#include "dev/leds.h"
#endif
#include "sys/clock.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#ifdef RF_LED_ENABLE
#define RF_RX_LED_ON()		leds_on(LEDS_RED);
#define RF_RX_LED_OFF()		leds_off(LEDS_RED);
#define RF_TX_LED_ON()		leds_on(LEDS_GREEN);
#define RF_TX_LED_OFF()		leds_off(LEDS_GREEN);
#else
#define RF_RX_LED_ON()
#define RF_RX_LED_OFF()
#define RF_TX_LED_ON()
#define RF_TX_LED_OFF()
#endif

#ifdef HAVE_RF_ERROR
uint8_t rf_error = 0;
#endif


#if !SHORTCUTS_CONF_NETSTACK
/*---------------------------------------------------------------------------*/
PROCESS(cc2430_rf_process, "CC2430 RF driver");
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2430_rf_process, ev, data)
{
  int len;
  PROCESS_BEGIN();
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    packetbuf_clear();
    len = cc2430_rf_read(packetbuf_dataptr(), PACKETBUF_SIZE);
    if(len > 0) {
      packetbuf_set_datalen(len);
      NETSTACK_RDC.input();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * RF interrupt service routine.
 *
 */
void
cc2430_rf_ISR( void ) __interrupt (RF_VECTOR)
{
  EA = 0;
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  /*
   * We only vector here if RFSTATUS.FIFOP goes high.
   * Just double check the flag.
   */
  if(RFIF & IRQ_FIFOP) {
      RF_RX_LED_ON();
      /* Poll the RF process which calls cc2430_rf_read() */
      process_poll(&cc2430_rf_process);
  }
  S1CON &= ~(RFIF_0 | RFIF_1);

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
  EA = 1;
}
#endif
/*---------------------------------------------------------------------------*/
#if CC2430_RFERR_INTERRUPT
/**
 * RF error interrupt service routine.
 * Turned off by default, can be turned on in contiki-conf.h
 */
void
cc2430_rf_error_ISR( void ) __interrupt (RFERR_VECTOR)
{
  EA = 0;
  TCON_RFERRIF = 0;
#ifdef HAVE_RF_ERROR
  rf_error = 254;
#endif
  cc2430_rf_command(ISRFOFF);
  cc2430_rf_command(ISFLUSHRX);
  cc2430_rf_command(ISFLUSHRX);
  cc2430_rf_command(ISRXON);
  RF_RX_LED_OFF();
  RF_TX_LED_OFF();
  EA = 1;
}
#endif
/*---------------------------------------------------------------------------*/
/**
 * Execute a single CSP command.
 *
 * \param command command to execute
 *
 */
void
cc2430_rf_command(uint8_t command)
{
  if (command >= 0xE0) { /*immediate strobe*/
    uint8_t fifo_count;
    switch (command) { /*hardware bug workaround*/
    case ISRFOFF:
    case ISRXON:
    case ISTXON:
      fifo_count = RXFIFOCNT;
      RFST = command;
      clock_delay(2);
      if (fifo_count != RXFIFOCNT) {
        RFST = ISFLUSHRX;
        RFST = ISFLUSHRX;
      }
      break;

    default:
      RFST = command;
    }
  } else if (command == SSTART) {
    RFIF &= ~IRQ_CSP_STOP; /*clear IRQ flag*/
    RFST = SSTOP; /*make sure there is a stop in the end*/
    RFST = ISSTART; /*start execution*/
    while ((RFIF & IRQ_CSP_STOP) == 0);
  } else {
    RFST = command; /*write command*/
  }
}
/*---------------------------------------------------------------------------*/
/* 
 * non-banked functions called through function pointers then call banked code
 */
static int
init(void)
{
  int rv = cc2430_rf_init();
#if !SHORTCUTS_CONF_NETSTACK
  process_start(&cc2430_rf_process, NULL);
#endif
  return rv;
}
/*---------------------------------------------------------------------------*/
static int
prepare(const void *data, unsigned short len)
{
  return cc2430_rf_prepare(data,len);
}
/*---------------------------------------------------------------------------*/
static int
transmit(unsigned short len)
{
  return cc2430_rf_transmit();
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  return cc2430_rf_rx_disable();
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return cc2430_rf_rx_enable();
}
/*---------------------------------------------------------------------------*/
static int
send(void *payload, unsigned short payload_len)
{
  cc2430_rf_prepare(payload, payload_len);
  return cc2430_rf_transmit();
}
/*---------------------------------------------------------------------------*/
static int
read(void *buf, unsigned short bufsize)
{
  return cc2430_rf_read(buf, bufsize);
}
/*---------------------------------------------------------------------------*/
static int
channel_clear(void)
{
  return cc2430_rf_cca_check(0, 0);
    }
/*---------------------------------------------------------------------------*/
static int
receiving_packet(void)
{
  return cc2430_rf_receiving_packet();
  }
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  return cc2430_rf_pending_packet();
}
/*---------------------------------------------------------------------------*/
const struct radio_driver cc2430_rf_driver =
  {
    init,
    prepare,
    transmit,
    send,
    read,
    channel_clear,
    receiving_packet,
    pending_packet,
    on,
    off,
  };
/*---------------------------------------------------------------------------*/
