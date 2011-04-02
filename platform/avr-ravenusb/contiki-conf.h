/*
 * Copyright (c) 2006, Technical University of Munich
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
 *
 * @(#)$$
 */

/**
 * \file
 *         Configuration for RZRAVEN USB stick "jackdaw"
 *
 * \author
 *         Simon Barner <barner@in.tum.de>
 *         David Kopf <dak664@embarqmail.com>
 */

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

/* ************************************************************************** */
//#pragma mark Basic Configuration
/* ************************************************************************** */

/* MCU and clock rate */
#define PLATFORM         PLATFORM_AVR
#define RAVEN_REVISION	 RAVENUSB_C
#ifndef F_CPU
#define F_CPU            8000000UL
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef int32_t  s32_t;
typedef unsigned char u8_t;
typedef unsigned short u16_t;
typedef unsigned long u32_t;
typedef unsigned short clock_time_t;
typedef unsigned short uip_stats_t;
typedef unsigned long off_t;

void clock_delay(unsigned int us2);
void clock_wait(int ms10);
void clock_set_seconds(unsigned long s);
unsigned long clock_seconds(void);

/* Maximum timer interval for 16 bit clock_time_t */
#define INFINITE_TIME 0xffff

/* Clock ticks per second */
#define CLOCK_CONF_SECOND 125

/* Maximum tick interval is 0xffff/125 = 524 seconds */
#define RIME_CONF_BROADCAST_ANNOUNCEMENT_MAX_TIME CLOCK_CONF_SECOND * 524UL /* Default uses 600UL */
#define COLLECT_CONF_BROADCAST_ANNOUNCEMENT_MAX_TIME CLOCK_CONF_SECOND * 524UL /* Default uses 600UL */

/* The LED scheme. Original scheme was blue=online, red=tx, green=rx, yellow=serial I/O
 * Alternate scheme uses a separate LED process and hooks for easier reconfiguration.
 * The default for the alternate is blue=serial, red=rx, green=tx, yellow=online(optional 50% duty)
 * The "ready" led can be dimmed through lowering the duty cycle in the main loop.
 */
#define JACKDAW_CONF_ALT_LED_SCHEME     1
#define JACKDAW_CONF_DIM_ONLINE_LED     1

/* Use EEPROM settings manager, or hard-coded EEPROM reads? */
/* Generate random MAC address on first startup? */
/* Random number from radio clock skew or ADC noise? */
#define JACKDAW_CONF_USE_SETTINGS		0
#define JACKDAW_CONF_RANDOM_MAC			0
#define RNG_CONF_USE_RADIO_CLOCK		1
//#define RNG_CONF_USE_ADC	1

/* COM port to be used for SLIP connection. Not tested on Jackdaw. */
#define SLIP_PORT RS232_PORT_0

/* Pre-allocated memory for loadable modules heap space (in bytes)*/
/* Default is 4096. Currently used only when elfloader is present. Not tested on Jackdaw */
//#define MMEM_CONF_SIZE 256

/* Starting address for code received via the codeprop facility. Not tested on Jackdaw */
//#define EEPROMFS_ADDR_CODEPROP 0x8000

/* ************************************************************************** */
//#pragma mark USB Ethernet Hooks
/* ************************************************************************** */
#if JACKDAW_CONF_ALT_LED_SCHEME
#define USB_HOOK_UNENUMERATED()		status_leds_unenumerated()
#define USB_ETH_HOOK_READY()		status_leds_ready()
#define USB_ETH_HOOK_INACTIVE()		status_leds_inactive()
#else
#define USB_ETH_HOOK_RX_START()	   rx_start_led()
#define USB_ETH_HOOK_TX_END()	   tx_end_led()
#define USB_HOOK_UNENUMERATED()
#define USB_ETH_HOOK_READY()
#define USB_ETH_HOOK_INACTIVE()		Led0_toggle()
#endif

#ifndef USB_ETH_HOOK_SET_PROMISCIOUS_MODE
#define USB_ETH_HOOK_SET_PROMISCIOUS_MODE(value)	rf230_set_promiscuous_mode(value)
#endif
#ifndef USB_ETH_HOOK_IS_READY_FOR_INBOUND_PACKET
#define	USB_ETH_HOOK_IS_READY_FOR_INBOUND_PACKET()		rf230_is_ready_to_send()
#endif
#ifndef USB_ETH_HOOK_HANDLE_INBOUND_PACKET
#define USB_ETH_HOOK_HANDLE_INBOUND_PACKET(buffer,len)	do { uip_len = len ; mac_ethernetToLowpan(buffer); } while(0)
#endif
#ifndef USB_ETH_HOOK_INIT
#define USB_ETH_HOOK_INIT()		mac_ethernetSetup()
#endif

/* ************************************************************************** */
//#pragma mark RF230BB Hooks
/* ************************************************************************** */
#if JACKDAW_CONF_ALT_LED_SCHEME
#define RF230BB_HOOK_RADIO_OFF() status_leds_radio_off()
#define RF230BB_HOOK_RADIO_ON()  status_leds_radio_on()
#else
#define RF230BB_HOOK_RADIO_OFF()
#define RF230BB_HOOK_RADIO_ON()
#endif
#define RF230BB_HOOK_TX_PACKET(buffer,total_len) mac_log_802_15_4_tx(buffer,total_len)
#define RF230BB_HOOK_RX_PACKET(buffer,total_len) mac_log_802_15_4_rx(buffer,total_len)
#define	RF230BB_HOOK_IS_SEND_ENABLED()	mac_is_send_enabled()
extern bool mac_is_send_enabled(void);
extern void mac_log_802_15_4_tx(const uint8_t* buffer, size_t total_len);
extern void mac_log_802_15_4_rx(const uint8_t* buffer, size_t total_len);


/* ************************************************************************** */
//#pragma mark USB CDC-ACM (UART) Hooks
/* ************************************************************************** */
#if JACKDAW_CONF_ALT_LED_SCHEME
#define USB_CDC_ACM_HOOK_RX(char)				status_leds_serial_rx()
#define USB_CDC_ACM_HOOK_TX_END(char)			status_leds_serial_tx()
#define USB_CDC_ACM_HOOK_CLS_CHANGED(state)		status_leds_serial_rx()
#define USB_CDC_ACM_HOOK_CONFIGURED()			status_leds_serial_rx()
#else
#define USB_CDC_ACM_HOOK_TX_END(char)			vcptx_end_led()
#define USB_CDC_ACM_HOOK_CLS_CHANGED(state)		vcptx_end_led()
#define USB_CDC_ACM_HOOK_CONFIGURED()			vcptx_end_led()
#endif

/* ************************************************************************** */
//#pragma mark Serial Port Settings
/* ************************************************************************** */
/* Set USB_CONF_MACINTOSH to prefer CDC-ECM+DEBUG enumeration for Mac/Linux 
 * Leave undefined to prefer RNDIS+DEBUG enumeration for Windows/Linux
 * The Mac configuration will still enumerate as RNDIS-ONLY
 * on Windows (without the serial port). You will have to point to the
 * INF folder to associate the different Mac VID/PID with RNDIS.
 * At present the Windows configuration will not enumerate on the Mac at all,
 * since it uses a custom descriptor for USB composite devices.
 */ 
#define USB_CONF_MACINTOSH       0

/* Set USB_CONF_SERIAL to enable the USB serial port that allows control of the
 * run-time configuration (COMx on Windows, ttyACMx on Linux, tty.usbmodemx on Mac)
 * Debug printfs will go to this port unless USB_CONF_RS232 is set.
 */
#define USB_CONF_SERIAL          1
 
/* RS232 debugs have less effect on network timing and are less likely
 * to be dropped due to buffer overflow.
 * The tx pad is the middle one behind the jackdaw leds.
 * RS232 output will work with or without enabling the USB serial port.
 * If USB serial not enabled the Jackdaw menu is available through the RS232 port.
 */
#define USB_CONF_RS232           0

/* Mass storage uses whatever program flash is left over for a USB thumb drive.
 * After initialization by the host (Prompt in Windows, use the disk utility in *nix)
 * the /cpu/avr/dev/usb/INF folder can be copied to it. On a Windows machine without
 * the RNDIS network or CDC-ACM serial driver association the thumb drive will enumerate
 * and the files copied over. Then replug the stick and point to the INF folder.
 * Note: Since the Windows-preferred and Macintosh-preferred builds have different serial
 * numbers, the drivers may have to loaded twice.
 */
#define USB_CONF_STORAGE         1

/* Jackdaw menu options */
#define JACKDAW_CONF_BOOTLOADER      1  //Jump to bootloader, if present
#define JACKDAW_CONF_WATCHDOGRESET   1  //Reboot using watchdog timeout
#define JACKDAW_CONF_WINDOWSSWITCH   1  //Re-enumerate as RNDIS network interface
#define JACKDAW_CONF_STORAGESWITCH   1  //Re-enumerate as mass storage device
#define JACKDAW_CONF_CONFIGURABLERDC 1  //On-the fly switch between different radio duty cycles
/* Sneeze mode is useful for testing CCA on other radios.
 * During sneezing, attempted radio access may hang the MCU and cause a watchdog reset.
 * The host interface, jackdaw menu and rf230_send routines are temporarily disabled to prevent this.
 * but some calls from an internal uip stack might get through, e.g. from CCA or low power protocols.
 * Temporarily disabling all the possible accesses would add considerable complication to the radio driver.
 * Continous broadcasting at high power may violate RF emission regulations in some countries.
 */
#define RF230_CONF_SNEEZER        1

/* Jackdaw menu 'm' reporting options */
#define SICSLOW_ETHERNET_CONF_UPDATE_USB_ETH_STATS  1
#define RF230_CONF_RADIOSTATS     1
#define CONFIG_STACK_MONITOR      1

/* ************************************************************************** */
//#pragma mark UIP Settings
/* ************************************************************************** */
/* Network setup. The new NETSTACK interface requires RF230BB (as does ip4) */
/* These mostly have no effect when the Jackdaw is a repeater (CONTIKI_NO_NET=1 using fakeuip.c) */

#if !RF230BB
#warning legacy RF230 driver no longer functional
#endif

#if UIP_CONF_IPV6
#define RIMEADDR_CONF_SIZE       8
#define UIP_CONF_ICMP6           1
#define UIP_CONF_UDP             1
#define UIP_CONF_TCP             0
#define UIP_CONF_IPV6_RPL        0
#define NETSTACK_CONF_NETWORK       sicslowpan_driver
#define SICSLOWPAN_CONF_COMPRESSION SICSLOWPAN_COMPRESSION_HC06
#else
/* ip4 should build but is thoroughly untested */
#define RIMEADDR_CONF_SIZE       2
#define NETSTACK_CONF_NETWORK    rime_driver
#endif /* UIP_CONF_IPV6 */

/* See uip-ds6.h */
#define UIP_CONF_DS6_NBR_NBU     2
#define UIP_CONF_DS6_DEFRT_NBU   2
#define UIP_CONF_DS6_PREFIX_NBU  3
#define UIP_CONF_DS6_ROUTE_NBU   2
#define UIP_CONF_DS6_ADDR_NBU    3
#define UIP_CONF_DS6_MADDR_NBU   0
#define UIP_CONF_DS6_AADDR_NBU   0

#define UIP_CONF_LL_802154       1
#define UIP_CONF_LLH_LEN         14
#define UIP_CONF_BUFSIZE		 UIP_LINK_MTU + UIP_LLH_LEN + 4   /* +4 for vlan on macosx */

/* 10 bytes per stateful address context - see sicslowpan.c */
/* Default is 1 context with prefix aaaa::/64 */
/* These must agree with all the other nodes or there will be a failure to communicate! */
#//define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS 1
#define SICSLOWPAN_CONF_ADDR_CONTEXT_0 {addr_contexts[0].prefix[0]=0xaa;addr_contexts[0].prefix[1]=0xaa;}
#define SICSLOWPAN_CONF_ADDR_CONTEXT_1 {addr_contexts[1].prefix[0]=0xbb;addr_contexts[1].prefix[1]=0xbb;}
#define SICSLOWPAN_CONF_ADDR_CONTEXT_2 {addr_contexts[2].prefix[0]=0x20;addr_contexts[2].prefix[1]=0x01;addr_contexts[2].prefix[2]=0x49;addr_contexts[2].prefix[3]=0x78,addr_contexts[2].prefix[4]=0x1d;addr_contexts[2].prefix[5]=0xb1;}

/* 211 bytes per queue buffer */
#define QUEUEBUF_CONF_NUM        8

/* 54 bytes per queue ref buffer */
#define QUEUEBUF_CONF_REF_NUM    2

#define UIP_CONF_MAX_CONNECTIONS 1
#define UIP_CONF_MAX_LISTENPORTS 1

#define UIP_CONF_IP_FORWARD      0
#define UIP_CONF_FWCACHE_SIZE    0

#define UIP_CONF_IPV6_CHECKS     1
#define UIP_CONF_IPV6_QUEUE_PKT  1
#define UIP_CONF_IPV6_REASSEMBLY 0

#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_TCP_SPLIT       0
#define UIP_CONF_STATISTICS      1

  /* Network setup */
#if 1              /* No radio cycling */
#define NETSTACK_CONF_MAC         nullmac_driver
#if JACKDAW_CONF_CONFIGURABLERDC
/* Configurable radio cycling */
/* EXPERIMENTAL */
struct rdc_driver;
extern const struct rdc_driver *rdc_config_driver;
#define NETSTACK_CONF_RDC         (*rdc_config_driver) 
#else
#define NETSTACK_CONF_RDC         nullrdc_driver
#endif
#define NETSTACK_CONF_FRAMER      framer_802154
#define NETSTACK_CONF_RADIO       rf230_driver
#define CHANNEL_802_15_4          26
/* AUTOACK receive mode gives better rssi measurements, even if ACK is never requested */
#define RF230_CONF_AUTOACK        1
/* Request 802.15.4 ACK on all packets sent by sicslowpan.c (else autoretry) */
/* Primarily for testing, since all broadcasts will be retransmitted (no one will ACK them) */
#define SICSLOWPAN_CONF_ACK_ALL   0
/* Number of auto retry attempts 0-15 (0 implies don't use extended TX_ARET_ON mode with CCA) */
#define RF230_CONF_AUTORETRIES    1
/* CCA theshold energy -91 to -61 dBm (default -77). Set this smaller than the expected minimum rssi to avoid packet collisions */
/* The Jackdaw menu 'm' command shows the smallest ever received rssi */
#define RF230_CONF_CCA_THRES    -85
/* Allow 6loWPAN fragmentation for larger payloads (more efficient on a reliable channel, more retransmits otherwise) */
#define SICSLOWPAN_CONF_FRAG      1
/* Timeout for fragment reassembly. A reissued browser GET will also cancel reassembly, typically in 2-3 seconds */
#define SICSLOWPAN_CONF_MAXAGE    3

#elif 0             /* Contiki-mac radio cycling */
#define NETSTACK_CONF_MAC         nullmac_driver
#define NETSTACK_CONF_RDC         contikimac_driver
#define NETSTACK_CONF_FRAMER      framer_802154
#define NETSTACK_CONF_RADIO       rf230_driver
#define CHANNEL_802_15_4          26
#define RF230_CONF_AUTOACK        0
#define RF230_CONF_AUTORETRIES    0
#define SICSLOWPAN_CONF_FRAG      1
#define SICSLOWPAN_CONF_MAXAGE    3

#elif 1             /* cx-mac radio cycling */
#define NETSTACK_CONF_MAC         nullmac_driver
//#define NETSTACK_CONF_MAC       csma_driver
#define NETSTACK_CONF_RDC         cxmac_driver
#define NETSTACK_CONF_FRAMER      framer_802154
#define NETSTACK_CONF_RADIO       rf230_driver
#define CHANNEL_802_15_4          26
#define RF230_CONF_AUTOACK        1
#define RF230_CONF_AUTORETRIES    1
#define SICSLOWPAN_CONF_FRAG      1
#define SICSLOWPAN_CONF_MAXAGE    3
#define CXMAC_CONF_ANNOUNCEMENTS    0
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8
#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM        8
#undef UIP_CONF_DS6_NBR_NBU
#define UIP_CONF_DS6_NBR_NBU       5
#undef UIP_CONF_DS6_ROUTE_NBU
#define UIP_CONF_DS6_ROUTE_NBU     5

#else
#error Network configuration not specified!
#endif   /* Network setup */


/* ************************************************************************** */
//#pragma mark RPL Settings
/* ************************************************************************** */

#if UIP_CONF_IPV6_RPL

/* Not completely working yet. Works on Ubuntu after $ifconfig usb0 -arp to drop the neighbor solitications */
/* Dropping the NS on other OSs is more complicated, see http://www.sics.se/contiki/wiki/index.php/Jackdaw_RNDIS_RPL_border_router */

/* RPL requires the uip stack. Change #CONTIKI_NO_NET=1 to UIP_CONF_IPV6=1 in the examples makefile,
   or include the needed source files in /plaftorm/avr-ravenusb/Makefile.avr-ravenusb */
/* For the present the buffer_length calcs in rpl-icmp6.c will need adjustment in three
   places by the length difference between 6lowpan (0) and ethernet (14) link-layer headers:
 // buffer_length = uip_len - uip_l2_l3_icmp_hdr_len;
    buffer_length = uip_len - uip_l2_l3_icmp_hdr_len + UIP_LLH_LEN; //Add jackdaw ethernet header
    The following define will do this hack automatically. */
#define RPL_CONF_ADD_FALLBACK_INTERFACE_HEADER_LENGTH 1

/* Define MAX_*X_POWER to reduce tx power and ignore weak rx packets for testing a miniature multihop network.
 * Leave undefined for full power and sensitivity.
 * tx=0 (3dbm, default) to 15 (-17.2dbm)
 * RF230_CONF_AUTOACK sets the extended mode using the energy-detect register with rx=0 (-91dBm) to 84 (-7dBm)
 *   else the rssi register is used having range 0 (91dBm) to 28 (-10dBm)
 *   For simplicity RF230_MIN_RX_POWER is based on the energy-detect value and divided by 3 when autoack is not set.
 * On the RF230 a reduced rx power threshold will not prevent autoack if enabled and requested.
 * These numbers applied to both Raven and Jackdaw give a maximum communication distance of about 15 cm
 * and a 10 meter range to a full-sensitivity RF230 sniffer.
#define RF230_MAX_TX_POWER 15
#define RF230_MIN_RX_POWER 30
 */

#define UIP_CONF_ROUTER             1
#define RPL_BORDER_ROUTER           1
#define RPL_CONF_STATS              0
#define UIP_CONF_BUFFER_SIZE	 1300
//#define UIP_CONF_DS6_NBR_NBU       12
//#define UIP_CONF_DS6_ROUTE_NBU     12
#undef UIP_FALLBACK_INTERFACE
#define UIP_FALLBACK_INTERFACE rpl_interface
#define UIP_CONF_ND6_SEND_RA		0
#define UIP_CONF_ND6_REACHABLE_TIME 600000
#define UIP_CONF_ND6_RETRANS_TIMER  10000

/* Save all the RAM we can */
#define PROCESS_CONF_NO_PROCESS_NAMES 1
#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM           2
#undef QUEUEBUF_CONF_REF_NUM
#define QUEUEBUF_CONF_REF_NUM       1
#undef UIP_CONF_TCP_SPLIT
#define UIP_CONF_TCP_SPLIT          0
#undef UIP_CONF_STATISTICS
#define UIP_CONF_STATISTICS         0
#undef UIP_CONF_IPV6_QUEUE_PKT
#define UIP_CONF_IPV6_QUEUE_PKT     0
#define UIP_CONF_PINGADDRCONF       0
#define UIP_CONF_LOGGING            0
#undef UIP_CONF_MAX_CONNECTIONS
#define UIP_CONF_MAX_CONNECTIONS    2
#undef UIP_CONF_MAX_LISTENPORTS
#define UIP_CONF_MAX_LISTENPORTS    2
#define UIP_CONF_UDP_CONNS          6

/* Optional, TCP needed to serve the RPL neighbor web page currently hard coded at bbbb::11 */
/* The RPL neighbors can also be viewed using the jack menu */
/* A small MSS is adequate for the internal jackdaw webserver and RAM is very limited*/
#define RPL_HTTPD_SERVER            0
#if RPL_HTTPD_SERVER
#undef UIP_CONF_TCP            
#define UIP_CONF_TCP                1
#define UIP_CONF_TCP_MSS           48
#define UIP_CONF_RECEIVE_WINDOW    48
#undef UIP_CONF_DS6_NBR_NBU
#define UIP_CONF_DS6_NBR_NBU        5
#undef UIP_CONF_DS6_ROUTE_NBU
#define UIP_CONF_DS6_ROUTE_NBU      5
#undef UIP_CONF_MAX_CONNECTIONS
#define UIP_CONF_MAX_CONNECTIONS    2
#endif

#define UIP_CONF_ICMP_DEST_UNREACH 1
#define UIP_CONF_DHCP_LIGHT
#undef UIP_CONF_FWCACHE_SIZE
#define UIP_CONF_FWCACHE_SIZE    30
#define UIP_CONF_BROADCAST       1
//#define UIP_ARCH_IPCHKSUM        1

/* Experimental option to pick up a prefix from host interface router advertisements */
/* Requires changes in uip6 and uip-nd6.c to pass link-local RA broadcasts */
/* If this is zero the prefix will be manually set in contiki-raven-main.c */
#define UIP_CONF_ROUTER_RECEIVE_RA  0

#endif /* UIP_CONF_IPV6_RPL */

/* ************************************************************************** */
//#pragma mark Other Settings
/* ************************************************************************** */

/* Use Atmel 'Route Under MAC', currently just in RF230 sniffer mode! */
/* Route-Under-MAC uses 16-bit short addresses */
//#define UIP_CONF_USE_RUM  1
#if UIP_CONF_USE_RUM
#undef  UIP_CONF_LL_802154
#define UIP_DATA_RUM_OFFSET      5
#endif /* UIP_CONF_USE_RUM */

#define CCIF
#define CLIF

#endif /* __CONTIKI_CONF_H__ */
