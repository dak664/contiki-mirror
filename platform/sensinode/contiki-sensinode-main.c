#include "contiki.h"
#include "sys/clock.h"
#include "sys/autostart.h"

#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/bus.h"
#include "dev/leds.h"
#include "dev/uart.h"
#include "dev/models.h"
#include "dev/cc2430_rf.h"
#include "dev/watchdog.h"
#include "net/rime.h"
#include "net/netstack.h"
#include "sensinode-debug.h"
#include "dev/watchdog-cc2430.h"
#include "dev/sensinode-sensors.h"
#include "contiki-lib.h"
#include "contiki-net.h"

static volatile int i, a;
unsigned short node_id = 0;			/* Manually sets MAC address when > 0 */

#if SHORTCUTS_CONF_NETSTACK
static int len;
#endif

#ifdef STARTUP_CONF_VERBOSE
#define STARTUP_VERBOSE STARTUP_CONF_VERBOSE
#else
#define STARTUP_VERBOSE 0
#endif

static __data int r;
/*---------------------------------------------------------------------------*/
static void
fade(int l)
{
  int k, j;
  for(k = 0; k < 400; ++k) {
    j = k > 200? 400 - k: k;

    leds_on(l);
    for(i = 0; i < j; ++i) {
      a = i;
    }
    leds_off(l);
    for(i = 0; i < 200 - j; ++i) {
      a = i;
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_rime_addr(void)
{
  rimeaddr_t addr;
  static uint8_t ft_buffer[8];
  uint8_t *addr_long = NULL;
  uint16_t addr_short = 0;
  unsigned char i;

#if SHORTCUTS_CONF_FLASH_READ
  /*
   * The MAC is always stored in 0x1FFF8 of our flash. This maps to address
   * 0xFFF8 of our CODE segment, when BANK3 is selected.
   * Switch to BANK3, read 8 bytes starting at 0xFFF8 and restore last BANK
   * Since we are called from main(), this MUST be BANK1 or something is very
   * wrong. This code can be used even without banking
   */
  __code unsigned char * macp;

  /* Don't interrupt us to make sure no BANK switching happens while working */
  DISABLE_INTERRUPTS();

  /* Switch to BANK3, map CODE: 0x8000 – 0xFFFF to FLASH: 0x18000 – 0x1FFFF */
  FMAP = 3;

  /* Set our pointer to the correct address and fetch 8 bytes of MAC */
  macp = (__code unsigned char *) 0xFFF8;

  for(i=0; i < 8; i++) {
    ft_buffer[i] = *macp;
    macp++;
  }

  /* Remap 0x8000 – 0xFFFF to BANK1 */
  FMAP = 1;
  ENABLE_INTERRUPTS();
#else
  /*
   * Or use the more generic flash_read() routine which can read from any
   * address of our flash
   */
  flash_read(&ft_buffer[0], 0x1FFF8, 8);
#endif

  /* MAC is stored LSByte first, so we print it backwards */
#if STARTUP_VERBOSE
  putstring("Read MAC from flash: ");
  for(i = 7; i > 0; --i) {
    puthex(ft_buffer[i]);
    putchar(':');
  }
  puthex(ft_buffer[0]);
  putchar('\n');
  putstring("Rime is 0x");
  puthex(sizeof(rimeaddr_t));
  putstring(" bytes long.\n");
#endif
  memset(&addr, 0, sizeof(rimeaddr_t));

  /* Flip the byte order and store MSB first */
#if UIP_CONF_IPV6
  for(i = 0; i < RIMEADDR_SIZE; ++i) {
    addr.u8[i] = ft_buffer[RIMEADDR_SIZE - 1 - i];
  }
#else
  if(node_id == 0) {
    for(i = 0; i < RIMEADDR_SIZE; ++i) {
      addr.u8[i] = ft_buffer[RIMEADDR_SIZE - 1 - i];
    }
  } else {
    putstring("Setting manual address from node_id\n");
    addr.u8[1] = node_id >> 8;
    addr.u8[0] = node_id & 0xff;
  }
#endif

  rimeaddr_set_node_addr(&addr);
  /* Now the address is stored MSB first */
#if STARTUP_VERBOSE
  putstring("Rime configured with address ");
  for(i = 0; i < RIMEADDR_SIZE - 1; i++) {
    puthex(addr.u8[i]);
    putchar(':');
  }
  puthex(addr.u8[i]);
  putchar('\n');
#endif

  /* Set the cc2430 RF addresses */
#if (RIMEADDR_SIZE==8)
	  addr_long = (uint8_t *) addr.u8;
#else
    addr_short = (addr.u8[0] * 256) + addr.u8[1];
#endif
  cc2430_rf_set_addr(0xffff, addr_short, addr_long);
}
/*---------------------------------------------------------------------------*/
int
main(void)
{

  /* Hardware initialization */
  bus_init();
  rtimer_init();

  /* Init UART1 without acknowledging the RX interrupt */
  uart1_init(115200, UART_RX_INT_OFF);

  /* model-specific h/w init. This will turn on UART interrupts if needed */
  model_init();

  /* Init LEDs here  for all other models (will do nothing for N740) */
  leds_init();
  fade(LEDS_GREEN);

  /* initialize process manager. */
  process_init();

#if SLIP_ARCH_CONF_ENABLE
  /* On cc2430, the argument is not used. We always use 115200 */
  slip_arch_init(0);
#else
  uart1_set_input(serial_line_input_byte);
  serial_line_init();
#endif

#ifdef HAVE_DMA
  dma_init();
#endif

#if STARTUP_VERBOSE
  putstring("##########################################\n");
#endif
  putstring("Welcome to " CONTIKI_VERSION_STRING ".\n");
  putstring("Running on: " SENSINODE_MODEL ".\n");

#if STARTUP_VERBOSE
#ifdef HAVE_SDCC_BANKING
  putstring("  With Banking.\n");
#endif /* HAVE_SDCC_BANKING */
#ifdef SDCC_MODEL_LARGE
  putstring("  --model-large\n");
#endif /* SDCC_MODEL_LARGE */
#ifdef SDCC_MODEL_HUGE
  putstring("  --model-huge\n");
#endif /* SDCC_MODEL_HUGE */
#ifdef SDCC_STACK_AUTO
  putstring("  --stack-auto\n");
#endif /* SDCC_STACK_AUTO */
#ifdef SDCC_USE_XSTACK
  putstring("  --xstack\n");
#endif /* SDCC_USE_XSTACK */

  putchar('\n');

  putstring(" Network: ");
  putstring(NETSTACK_NETWORK.name);
  putchar('\n');
  putstring(" MAC: ");
  putstring(NETSTACK_MAC.name);
  putchar('\n');
  putstring(" RDC: ");
  putstring(NETSTACK_RDC.name);
  putchar('\n');

  putstring("##########################################\n");
#endif /* STARTUP_VERBOSE */

  watchdog_init();

  /* Initialise the cc2430 RNG engine. */
  random_init(0);

  /* initialize the netstack */
  netstack_init();
  set_rime_addr();

  /* start services */
  process_start(&etimer_process, NULL);
  ctimer_init();

#if BUTTON_SENSOR_ON || ADC_SENSOR_ON
  process_start(&sensors_process, NULL);
  sensinode_sensors_activate();
#endif

#if UIP_CONF_IPV6
  memcpy(&uip_lladdr.addr, &rimeaddr_node_addr, sizeof(uip_lladdr.addr));
  queuebuf_init();
  process_start(&tcpip_process, NULL);

#if (!UIP_CONF_IPV6_RPL)
  {
    uip_ipaddr_t ipaddr;

    uip_ip6addr(&ipaddr, 0x2001, 0x630, 0x301, 0x6453, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
  }
#endif /* UIP_CONF_IPV6_RPL */
#endif /* UIP_CONF_IPV6 */

  fade(LEDS_RED);

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  autostart_start(autostart_processes);
  watchdog_start();

  while(1) {
    do {
      /* Reset watchdog and handle polls and events */
      watchdog_periodic();
      r = process_run();
    } while(r > 0);
#if SHORTCUTS_CONF_NETSTACK
    len = NETSTACK_RADIO.pending_packet();
    if(len) {
      packetbuf_clear();
      len = cc2430_rf_read(packetbuf_dataptr(), PACKETBUF_SIZE);
      if(len > 0) {
        packetbuf_set_datalen(len);
        NETSTACK_RDC.input();
      }
    }
#endif
  }
}
/*---------------------------------------------------------------------------*/
