#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include "8051def.h"
#include "sys/cc.h"
#include "dev/banked.h"
#include <string.h>

/* Include Project Specific conf */
#ifdef PROJECT_CONF_H
#include "project-conf.h"
#endif /* PROJECT_CONF_H */

/*
 * To help syntax checkers with parsing of sdcc keywords. It basically defines
 * empty macros for keywords such as __sfr etc. Does nothing when compiling
 */
#include <lint.h>

/* Time type. */
/*typedef unsigned long clock_time_t;*/
typedef unsigned short clock_time_t;

/* Defines tick counts for a second. */
#define CLOCK_CONF_SECOND		128

/* Memory filesystem RAM size. */
#define CFS_RAM_CONF_SIZE		512

/* Logging.. */
#define LOG_CONF_ENABLED		0

/* Energest Module */
#ifndef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON      0
#endif

/* Verbose Startup? Turning this off saves 700+ bytes of CODE in HOME */
#define STARTUP_CONF_VERBOSE  0

/* More CODE space savings by turning off process names */
#define PROCESS_CONF_NO_PROCESS_NAMES 1

/*
 * UARTs: 1=>Enabled, 0=>Disabled. Default: Both Disabled (see uart.h)
 * Disabling UART0 saves ~200 bytes of CODE.
 * Disabling UART1 saves ~500 bytes of CODE but also disables all debugging
 * output. Should be used when nodes are meant to run on batteries
 *
 * On N740, by enabling UART1, you are also enabling an ugly hack which aims
 * to detect the USB connection during execution. It will then turn on/off
 * UART1 RX interrupts accordingly. This seems to work but you have been warned
 * If you start seeing random crashes when on battery, this is where to look.
 */
#ifndef UART_ONE_CONF_ENABLE
#define UART_ONE_CONF_ENABLE  1
#endif
#ifndef UART_ONE_CONF_WITH_INPUT
#define UART_ONE_CONF_WITH_INPUT 0
#endif
#define UART_ZERO_CONF_ENABLE 0

/* Are we a SLIP bridge? */
#if SLIP_ARCH_CONF_ENABLE
/* Make sure UART1 is enabled, with interrupts */
#undef UART_ONE_CONF_ENABLE
#undef UART_ONE_CONF_WITH_INPUT
#define UART_ONE_CONF_ENABLE  1
#define UART_ONE_CONF_WITH_INPUT 1
#define UIP_FALLBACK_INTERFACE slip_interface
#endif

/* Code Shortcuts */
/*
 * When set, the RF driver is no longer a contiki process and the RX ISR is
 * disabled. Instead of polling the radio process when data arrives, we
 * periodically check for data by directly invoking the driver from main()

 * When set, this directive also configures the following bypasses:
 *   - process_post_synch() in tcpip_input() (we call packet_input())
 *   - process_post_synch() in tcpip_uipcall (we call the relevant pthread)
 *   - mac_call_sent_callback() is replaced with sent() in various places
 *
 * These are good things to do, we reduce stack usage, RAM size and code size
 */
#define SHORTCUTS_CONF_NETSTACK   1

/*
 * Directly read mac from flash with a __code pointer, instead of using the
 * generic flash_read() routine. This reduces HOME code size
 */
#define SHORTCUTS_CONF_FLASH_READ 1

/*
 * Sensors
 * It is harmless to #define XYZ 1
 * even if the sensor is not present on our device
 */
#ifndef BUTTON_SENSOR_CONF_ON
#define BUTTON_SENSOR_CONF_ON   1  /* Buttons */
#endif
/* ADC - Turning this off will disable everything below */
#ifndef ADC_SENSOR_CONF_ON
#define ADC_SENSOR_CONF_ON      1
#endif
#define TEMP_SENSOR_CONF_ON     1  /* Temperature */
#define BATTERY_SENSOR_CONF_ON  1  /* Battery */
#define VDD_SENSOR_CONF_ON      1  /* Supply Voltage */
#define ACC_SENSOR_CONF_ON      1  /* Accelerometer */
#define ACC_SENSOR_CONF_GSEL    0  /* Acc. g-Select => 1: +/-11g, 0: +/-3g */
#define LIGHT_SENSOR_CONF_ON    1  /* Light */

/* Watchdog */
#define WDT_CONF_INTERVAL     0
#define WDT_CONF_TIMER_MODE   0 /* 0 or undefined for watchdog mode */

/* XXX argh, ugly hack to make stuff compile! */
#define snprintf(BUF, SIZE, ...) sprintf(BUF, __VA_ARGS__)

/* Network Stack */
#if UIP_CONF_IPV6
#define NETSTACK_CONF_NETWORK sicslowpan_driver
#ifndef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     sicslowmac_driver
#endif
#else
#define NETSTACK_CONF_NETWORK rime_driver
#ifndef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     sicslowmac_driver
#endif
#endif

#ifndef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     csma_driver
#endif
#define NETSTACK_CONF_RADIO   cc2430_rf_driver

/* #define CC2420_CONF_AUTOACK                  1  // this is useful. check for similar in cc2430 */
/* MDMCTRL0L.AUTOACK p165 in datasheet */
#if UIP_CONF_IPV6
/* Addresses, Sizes and Interfaces */
#define RIMEADDR_CONF_SIZE                   8  /* 8-byte addresses here, 2 otherwise */
#define UIP_CONF_LL_802154                   1
#define UIP_CONF_LLH_LEN                     0
#define UIP_CONF_NETIF_MAX_ADDRESSES         3

/* TCP, UDP, ICMP */
#define UIP_CONF_TCP                         0
#define UIP_CONF_UDP                         1
#define UIP_CONF_UDP_CHECKSUMS               1
#define UIP_CONF_ICMP6                       1
#define UIP_CONF_BROADCAST                   1

/* ND and Routing */
#define UIP_CONF_ROUTER                      1 
#define UIP_CONF_IPV6_RPL                    1
#define UIP_CONF_ND6_SEND_RA                 0
#define UIP_CONF_IP_FORWARD                  0
#define RPL_CONF_STATS                       0

#define UIP_CONF_ND6_REACHABLE_TIME     600000
#define UIP_CONF_ND6_RETRANS_TIMER       10000
#define UIP_CONF_DS6_NBR_NBU                 4 /* Handle 4 Neighbors */
#define UIP_CONF_DS6_ROUTE_NBU               4 /* Handle 4 Routes */

/* uIP */
#define UIP_CONF_BUFFER_SIZE               240
#define UIP_CONF_IPV6_QUEUE_PKT              0
#define UIP_CONF_IPV6_CHECKS                 1
#define UIP_CONF_IPV6_REASSEMBLY             0

/* 6lowpan */
#define SICSLOWPAN_CONF_COMPRESSION          SICSLOWPAN_COMPRESSION_HC06
#define SICSLOWPAN_CONF_FRAG                 0 /* About 2KB of CODE if 1 */
#define SICSLOWPAN_CONF_MAXAGE               8

/* Define our IPv6 prefixes/contexts here */
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS    1
#define SICSLOWPAN_CONF_ADDR_CONTEXT_0 { \
  addr_contexts[0].prefix[0] = 0x20; \
  addr_contexts[0].prefix[1] = 0x01; \
  addr_contexts[0].prefix[2] = 0x06; \
  addr_contexts[0].prefix[3] = 0x30; \
  addr_contexts[0].prefix[4] = 0x03; \
  addr_contexts[0].prefix[5] = 0x01; \
  addr_contexts[0].prefix[6] = 0x64; \
  addr_contexts[0].prefix[7] = 0x53; \
}

#define MAC_CONF_CHANNEL_CHECK_RATE          8
#define QUEUEBUF_CONF_NUM                    8
#else /* UIP_CONF_IPV6 */
/* Network setup for non-IPv6 (rime). */
#define UIP_CONF_IP_FORWARD                  1
#define UIP_CONF_BUFFER_SIZE               108
#define RIME_CONF_NO_POLITE_ANNOUCEMENTS     0
#define QUEUEBUF_CONF_NUM                    8
#endif /* UIP_CONF_IPV6 */

#endif /* __CONTIKI_CONF_H__ */
