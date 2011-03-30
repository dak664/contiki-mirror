/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file cdc_task.c **********************************************************
 *
 * \brief
 *      Manages the CDC-ACM Virtual Serial Port Dataclass for the USB Device
 *
 * \addtogroup usbstick
 *
 * \author 
 *        Colin O'Flynn <coflynn@newae.com>
 *        David Kopf <dak664@embartmail.com>
 *        Robert Quattlebaum <darco@deepdarc.com>
 *
 ******************************************************************************/
/* Copyright (c) 2008  ATMEL Corporation
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/
/**
 \ingroup usbstick
 \defgroup cdctask CDC Task
 @{
 */

//_____  I N C L U D E S ___________________________________________________


#include "contiki.h"
#include "usb_drv.h"
#include "usb_descriptors.h"
#include "usb_specific_request.h"
#include "cdc_task.h"
#include "serial/uart_usb_lib.h"
#include "rndis/rndis_protocol.h"
#include "rndis/rndis_task.h"
#include "sicslow_ethernet.h"
#include "rf230bb.h"

#include <stdio.h>
#include <stdlib.h>
#include "dev/watchdog.h"
#include "rng.h"

#include "bootloader.h"

#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define PRINTA(FORMAT,args...) printf_P(PSTR(FORMAT),##args)

#if JACKDAW_CONF_USE_SETTINGS
#include "settings.h"
#define settings_print(x) x==SETTINGS_STATUS_OK ? PRINTA(", and stored in EEPROM.\n") : PRINTA(", but error writing it to EEPROM\n")
#define settings_printerror PRINTA(", but error writing it to EEPROM\n")
#else
#define SETTINGS_STATUS_OK 1
#define settings_print(x) x==SETTINGS_STATUS_OK ? PRINTA(", not saved in EEPROM.\n") : PRINTA(", but error writing it to EEPROM\n")
#define settings_set_uint8(...) SETTINGS_STATUS_OK
#endif

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

//_____ M A C R O S ________________________________________________________


#define bzero(ptr,size)	memset(ptr,0,size)

//_____ D E F I N I T I O N S ______________________________________________


#define IAD_TIMEOUT_DETACH 300
#define IAD_TIMEOUT_ATTACH 600

//_____ D E C L A R A T I O N S ____________________________________________


void menu_print(void);
void menu_process(char c);

extern char usb_busy;

//! Counter for USB Serial port
extern U8    tx_counter;

#if !JACKDAW_CONF_ALT_LED_SCHEME
//! Timers for LEDs
uint8_t led3_timer;
#endif

//! previous configuration
static uint8_t previous_uart_usb_control_line_state = 0;


static uint8_t timer = 0;
static struct etimer et;

#define CONVERTTXPOWER 1
#if CONVERTTXPOWER
/**
 * \brief Convert tx power to dBm for display
 *
 * Printing RF23x tx power in dBm adds ~120 bytes to program flash size
 * Otherwise the numerical value of the tx power register is shown
 * 0=+3dBm, 15=-17.2dBm in a roughly logarithmic fashion.
 */
const char txonesdigit[16]   PROGMEM = {'3','2','2','1','1','0','0','1','2','3','4','5','7','9','2','7'};
const char txtenthsdigit[16] PROGMEM = {'0','6','1','6','1','5','2','2','2','2','2','2','2','2','2','2'};
static void printtxpower(void) {
    uint8_t power=rf230_get_txpower()&0xf;
    char sign=(power<6?'+':'-');
    char tens=(power>14?'1':'0');
    char ones=pgm_read_byte(&txonesdigit[power]);
    char tenths=pgm_read_byte(&txtenthsdigit[power]);
    if (tens=='0') {tens=sign;sign=' ';}
    PRINTA("%c%c%c.%cdBm",sign,tens,ones,tenths);
}
#endif

PROCESS(cdc_process, "Jackdaw Menu");

/**
 * \brief Communication Data Class (CDC) Process
 *
 *   This is the link between USB or RS232 serial I/O and the Jackdaw menu.
 *   USB serial input is used if enabled and enumerated, else RS232 if enabled.
 *   The jackdaw menu is output on whichever port is used for input.
 *   Debug prints can go to either, RS232 preferred, or can be turned off.
 */
#if USB_CONF_SERIAL
	static FILE *usb_stdout;
#if USB_CONF_RS232
	static FILE *rs232_stdout;
	static volatile unsigned char rs232_input_char,buffered_rs232_input_char;
int cdc_task_rs232in(unsigned char c) {rs232_input_char=c;return 0;}
#endif
#else
#define uart_usb_flush(...)
#endif

PROCESS_THREAD(cdc_process, ev, data_proc)
{
	PROCESS_BEGIN();
	
#if USB_CONF_RS232
	rs232_stdout=stdout;					//Save what should be rs232 stdout on entry
#endif

	while(1) {								//loop forever

#if USB_CONF_SERIAL							//USB serial port is enabled

#if !JACKDAW_CONF_ALT_LED_SCHEME
	    // turn off LED's if necessary
		if (led3_timer) led3_timer--;
		else			Led3_off();
#endif
		
 		if(Is_device_enumerated()) {		//and enumerated
			// If the configuration is different than the last time we checked...
			if((uart_usb_get_control_line_state()&1)!=previous_uart_usb_control_line_state) {
				previous_uart_usb_control_line_state = uart_usb_get_control_line_state()&1;
				static FILE* previous_stdout;
				
				if(previous_uart_usb_control_line_state&1) {
					previous_stdout = stdout;
					uart_usb_init();
					uart_usb_set_stdout();  //use it for jackdaw menu output
				//	menu_print(); do this later
				} else {
					stdout = previous_stdout;
				}
				usb_stdout=stdout;			//save the usb output port
			}

			//Flush buffer if timeout
	        if(timer >= 4 && tx_counter!=0 ){
	            timer = 0;
	            uart_usb_flush();
	        } else {
				timer++;
			}

			stdout=usb_stdout;				//switch to usb output for jackdaw menu
			while (uart_usb_test_hit()){    //process input characters
  		  	   menu_process(uart_usb_getchar());
            }

            if (usbstick_mode.debugOn) {
#if USB_CONF_RS232
			  stdout=rs232_stdout;			//switch back to rs232 for debugging
#endif
			} else {
			  stdout=NULL;					//else turn off debugging completely
			}
			
		} else {  							//USB enabled but not enumerated,
#if USB_CONF_RS232
			while (rs232_input_char) {		//take characters from rs232,
				buffered_rs232_input_char=rs232_input_char;
				rs232_input_char=0;			//Allow another char to come in while processing this one
				menu_process(buffered_rs232_input_char); 
			}
#endif
		}  //if (Is_device_enumerated())
       
#elif USB_CONF_RS232						//USB not enabled, but serial is
		stdout=rs232_stdout;				//restore jackdaw menu prints
		while (rs232_input_char) {			//take characters from rs232,
			buffered_rs232_input_char=rs232_input_char;
			rs232_input_char=0;				//Allow another char to come in while processing this one
			menu_process(buffered_rs232_input_char); 
		}
		if (usbstick_mode.debugOn==0) {
			stdout=NULL;					//disable debug prints
		}
#endif /* USB_CONF_SERIAL or USB_CONF_RS232 */


		if (USB_CONFIG_HAS_DEBUG_PORT(usb_configuration_nb)) {
			etimer_set(&et, CLOCK_SECOND/80);
		} else {
	//		etimer_set(&et, CLOCK_SECOND);
						etimer_set(&et, CLOCK_SECOND/80);
		}

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));	
		
	} // while(1)

	PROCESS_END();
}

/**
 \brief Print Jackdaw menu
 */
void menu_print(void)
{
	PRINTA("\n********* Jackdaw Menu ********\n");
	PRINTA("     [Built "  __DATE__ "]     \n");
	PRINTA("*  m     Print current mode   *\n");
	PRINTA("*  s     Toggle sniffer mode  *\n");
//	PRINTA("*  n     Set to network mode  *\n");
	PRINTA("*  6     Toggle 6LoWPAN       *\n");
	PRINTA("*  r     Toggle raw mode      *\n");
#if USB_CONF_RS232
	PRINTA("*  d     Toggle RS232 output  *\n");
#endif
	PRINTA("*  c     Set RF channel       *\n");
	PRINTA("*  p     Set RF power         *\n");
	PRINTA("*  e     Energy Scan          *\n");
#if JACKDAW_CONF_USE_SETTINGS
	PRINTA("*  E     Dump EEPROM settings *\n");
#endif
#if RF230BB && RF230_CONF_SNEEZER
	PRINTA("*  S     Enable sneezer mode  *\n");
#endif
#if UIP_CONF_IPV6_RPL
	PRINTA("*  N     RPL Neighbors        *\n");
	PRINTA("*  G     RPL Global Repair    *\n");
#endif
#if USB_CONF_STORAGE && USB_CONF_STORAGESWITCH
	PRINTA("*  U     Mount as mass-storage*\n");
#endif
#if USB_CONF_WINDOWSSWITCH
	PRINTA("*  W     Switch to RNDIS      *\n");
#endif
#if USB_CONF_BOOTLOADER
	if(bootloader_is_present())
	PRINTA("*  F     DFU Firmware Update  *\n");
#endif
#if USB_CONF_WATCHDOGRESET
	PRINTA("*  R     Reset (via WDT)      *\n");
#endif
	PRINTA("*  h,?   Print this menu      *\n");
	PRINTA("*** Press a key at any time ***\n");
}

#if UIP_CONF_IPV6_RPL
static void
ipaddr_add(const uip_ipaddr_t *addr)
{
  uint16_t a;
  int8_t i, f;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) PRINTA("::");
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        PRINTA(":");
      }
      PRINTA("%x",a);
    }
  }
}
#endif

/**
 \brief Process incomming char on Jackdaw menu port
 */
void menu_process(char c)
{

	static enum menustate_enum            /* Defines an enumeration type    */
	{
		normal,
		channel,
		txpower,
		confirmwipe,
		confirmdfuboot
	} menustate = normal;
	
	static char input_string[4];
	static uint8_t input_index;
	int input_value;

	if (menustate != normal) {

		switch(c) {
			case '\r':
			case '\n':		
								
				if (input_index)  {
					input_string[input_index] = 0;
					input_value = atoi(input_string);

					if (menustate==channel) {
						if (rf230_set_channel(input_value)) {
							PRINTA("\nChannel changed to %d",input_value);
							settings_print(settings_set_uint8(SETTINGS_KEY_CHANNEL, input_value));   
						} else {
							PRINTA("\nInvalid channel!\n");
						}

					} else if (menustate==txpower) {
				//		PRINTA(" "); //for some reason needs a print here to clear the string input...
						if (rf230_set_txpower(input_value)) {
							PRINTA("\nTransmit power changed to %d",input_value);
#if CONVERTTXPOWER
                            PRINTA(" [");printtxpower();PRINTA("]");
#endif
							settings_print(settings_set_uint8(SETTINGS_KEY_TXPOWER, input_value));
						} else {
							PRINTA("\nInvalid power level!\n");
						}
#if JACKDAW_CONF_USE_SETTINGS
					} else if (menustate == confirmwipe) {
						if ((input_string[0]=='y') || (input_string[0]=='Y')) {
							PRINTA("Wiping all settings. . .\n");
							settings_wipe();
							PRINTA("Done.\n");
						} else {
							PRINTA("\n");
						}
#endif

#if USB_CONF_BOOTLOADER
					} else if (menustate == confirmdfuboot) {
						if ((input_string[0]=='y') || (input_string[0]=='Y')) {
							uint8_t i;
							PRINTA("Entering DFU Mode...\n");
							uart_usb_flush();
							Leds_on();
							for(i = 0; i < 10; i++)_delay_ms(100);
							Leds_off();
							Jump_To_Bootloader();
						}
						PRINTA("\n");
#endif
					}
				} else {
					if (menustate==channel) {
						PRINTA("\nChannel unchanged.\n");
					} else if (menustate==txpower) {
						PRINTA("\nTransmit power unchanged.\n");
					}
				}
					
				menustate = normal;
				input_index = 0;
				break;
		
			case '\b':
			
				if (input_index) {
					input_index--;
					PRINTA("\b \b");
				}
				break;
#if 0
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
#endif
			default:
				if (input_index > 2) {  // No more than two digits at present. Beep if more.					
					putc('\a', stdout);
				} else {
					putc(c, stdout);				
					input_string[input_index++] = c;
				}
				break;

	//		default:

	//		break;
		}

 
	} else {  //menustate == normal

		uint8_t i;

        /* Most attempts to read RF230 status in sneeze mode (e.g. rssi) will hang the MCU */
        /* So convert any command into a sneeze off */
        if (usbstick_mode.sneeze) c='S';

		switch(c) {
			case '\r':
			case '\n':
				break;

			case 'h':
			case '?':
				menu_print();
				break;
			case '-':
				PRINTA("Bringing interface down\n");
				usb_eth_set_active(0);
				break;
			case '=':
			case '+':
				PRINTA("Bringing interface up\n");
				usb_eth_set_active(1);
				break;
#if JACKDAW_CONF_RANDOM_MAC
			case 'T':
				// Test "strong" random number generator of R Quattlebaum
                // This can potentially reboot the stick!
				PRINTA("RNG Output: ");
				watchdog_periodic();
				{
					uint8_t value = rng_get_uint8();
					uint8_t i;
					for(i=0;i<8;i++) {
					    putc(((value>>(7-i))&1)?'1':'0', stdout);
//						uart_usb_putchar(((value>>(7-i))&1)?'1':'0');
					}
					PRINTA("\n"));
					uart_usb_flush();
					watchdog_periodic();
				}
				break;
#endif
			case 's':
			   if (usbstick_mode.sendToRf==0) {
					PRINTA("Jackdaw now in network mode\n");
					usbstick_mode.sendToRf = 1;
					usbstick_mode.translate = 1;
					rf230_set_channel(rf230_get_channel());		//receive only addressed packets
				} else {
					PRINTA("Jackdaw now in sniffer mode\n");
					usbstick_mode.sendToRf = 0;
					usbstick_mode.translate = 0;
					rf230_listen_channel(rf230_get_channel()); 	//receive all packets
				}
				break;

			case '6':
				usbstick_mode.sicslowpan = !usbstick_mode.sicslowpan;
				PRINTA("Jackdaw will%s perform 6LoWPAN translation\nm",usbstick_mode.sicslowpan?"":" not");
				break;

			case 'r':
				usbstick_mode.raw = !usbstick_mode.raw;
				PRINTA("Jackdaw will%s capture raw frames\n",usbstick_mode.raw?"":" not");
				break;

#if RF230BB && RF230_CONF_SNEEZER
 			case 'S':
 				if (usbstick_mode.sneeze) {
					rf230_warm_reset();
					PRINTA("Jackdaw now behaving itself.\n");
					usbstick_mode.sneeze = 0;
				} else {
					if (rf230_get_txpower()<5)
						PRINTA("*****WARNING You are broadcasting at high power*******\n");
					rf230_start_sneeze();
					PRINTA("********Jackdaw is continuously broadcasting*******\n");
#if CONVERTTXPOWER
					PRINTA("*********on channel %2d with power ",rf230_get_channel());
					printtxpower();
					PRINTA("*********\n");
#else
					PRINTA("************on channel %2d with power %2d************\n"),rf230_get_channel(),rf230_get_txpower());
#endif
					PRINTA("Press any key to stop.\n");
					usbstick_mode.sneeze = 1;
				}
				break;
#endif

#if USB_CONF_RS232
			case 'd':
				usbstick_mode.debugOn = !usbstick_mode.debugOn;
				PRINTA("Jackdaw will%s ouput debug strings\n",usbstick_mode.debugOn?"":" not");
				break;
#endif

#if JACKDAW_CONF_USE_SETTINGS
			case 'E':
				settings_debug_dump(stdout);
				break;
			case 'D':
				PRINTA("\nDelete all eeprom settings [n]?");
				menustate = confirmwipe;
				break;
#endif

			case 'c':
				PRINTA("\nSelect 802.15.4 Channel in range 11-26 [%d]: ", rf230_get_channel());
				menustate = channel;
				break;

			case 'p':
				PRINTA("\nSelect transmit power (0=+3dBm 15=-17.2dBm) [%d]: ", rf230_get_txpower());
				menustate = txpower;
				break;

#if JACKDAW_CONF_USE_CONFIGURABLE_RDC
extern void jackdaw_choose_rdc_driver(uint8_t i);
			case '1':
				jackdaw_choose_rdc_driver(0);
				PRINTA("RDC Driver Changed To: %s\n", NETSTACK_CONF_RDC.name);
				break;
			case '2':
				jackdaw_choose_rdc_driver(1);
				PRINTA("RDC Driver Changed To: %s\n", NETSTACK_CONF_RDC.name);
				break;
			case '3':
				jackdaw_choose_rdc_driver(2);
				PRINTA("RDC Driver Changed To: %s\n", NETSTACK_CONF_RDC.name);
				break;
			case '4':
				jackdaw_choose_rdc_driver(3);
				PRINTA"RDC Driver Changed To: %s\n", NETSTACK_CONF_RDC.name);
				break;
#endif

#if UIP_CONF_IPV6_RPL
#include "rpl.h"
extern uip_ds6_nbr_t uip_ds6_nbr_cache[];
extern uip_ds6_route_t uip_ds6_routing_table[];
extern uip_ds6_netif_t uip_ds6_if;
			case 'N':
			{	uint8_t i,j;
				PRINTA("\nAddresses [%u max]\n",UIP_DS6_ADDR_NB);
				for (i=0;i<UIP_DS6_ADDR_NB;i++) {
					if (uip_ds6_if.addr_list[i].isused) {	  
						ipaddr_add(&uip_ds6_if.addr_list[i].ipaddr);
						PRINTA("\n"));
					}
				}
				PRINTA("\nNeighbors [%u max]\n",UIP_DS6_NBR_NB);
				for(i = 0,j=1; i < UIP_DS6_NBR_NB; i++) {
					if(uip_ds6_nbr_cache[i].isused) {
						ipaddr_add(&uip_ds6_nbr_cache[i].ipaddr);
						PRINTA("\n");
						j=0;
					}
				}
				if (j) PRINTA("  <none>");
				PRINTA("\nRoutes [%u max]\n",UIP_DS6_ROUTE_NB);
				for(i = 0,j=1; i < UIP_DS6_ROUTE_NB; i++) {
					if(uip_ds6_routing_table[i].isused) {
						ipaddr_add(&uip_ds6_routing_table[i].ipaddr);
						PRINTA("/%u (via ", uip_ds6_routing_table[i].length);
						ipaddr_add(&uip_ds6_routing_table[i].nexthop);
						if(uip_ds6_routing_table[i].state.lifetime < 600) {
							PRINTA(") %lus\n", uip_ds6_routing_table[i].state.lifetime);
						} else {
							PRINTA(")\n");
						}
						j=0;
					}
				}
				if (j) PRINTA("  <none>");
				PRINTA("\n---------\n"));
				break;
			}
			
			case 'G':
				PRINTA("Global repair returns %d\n",rpl_repair_dag(rpl_get_dag(RPL_ANY_INSTANCE))); 
				break;
            
            case 'L':
                rpl_local_repair(rpl_get_dag(RPL_ANY_INSTANCE));
                 PRINTA("Local repair initiated\n"); 
                 break;
 
            case 'Z':     //zap the routing table           
            {   uint8_t i; 
				for (i = 0; i < UIP_DS6_ROUTE_NB; i++) {
					uip_ds6_routing_table[i].isused=0;
                }
                PRINTA("Routing table cleared!\n"); 
                break;
            }
#endif				
			
			case 'm':
				PRINTA("Currently Jackdaw:\n  * Will ");
				if (usbstick_mode.sendToRf == 0) { PRINTA("not ");}
				PRINTA("send data over RF\n\r  * Will ");
				if (usbstick_mode.translate == 0) { PRINTA("not ");}
				PRINTA("change link-local addresses inside IP messages\n  * Will ");
				if (usbstick_mode.sicslowpan == 0) { PRINTA("not ");}
				PRINTA("decompress 6lowpan headers\n  * Will ");
				if (usbstick_mode.raw == 0) { PRINTA("not ");}

#if USB_CONF_RS232
				PRINTA("Output raw 802.15.4 frames\n\r  * Will ");
				if (usbstick_mode.debugOn == 0) { PRINTA("not ");}
				PRINTA("Output RS232 debug strings\n");
#else
				PRINTA("Output raw 802.15.4 frames\n");
#endif

				PRINTA("  * USB Ethernet MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
					((uint8_t *)&usb_ethernet_addr)[0],
					((uint8_t *)&usb_ethernet_addr)[1],
					((uint8_t *)&usb_ethernet_addr)[2],
					((uint8_t *)&usb_ethernet_addr)[3],
					((uint8_t *)&usb_ethernet_addr)[4],
					((uint8_t *)&usb_ethernet_addr)[5]
				);
				extern uint64_t macLongAddr;
				PRINTA("  * 802.15.4 EUI-64: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
					((uint8_t *)&macLongAddr)[0],
					((uint8_t *)&macLongAddr)[1],
					((uint8_t *)&macLongAddr)[2],
					((uint8_t *)&macLongAddr)[3],
					((uint8_t *)&macLongAddr)[4],
					((uint8_t *)&macLongAddr)[5],
					((uint8_t *)&macLongAddr)[6],
					((uint8_t *)&macLongAddr)[7]
				);
#if UIP_CONF_IPV6_RPL
				PRINTA("  * Suports RPL mesh routing\n");
#endif
#if CONVERTTXPOWER
				PRINTA("  * Operates on channel %d with TX power ",rf230_get_channel());
				printtxpower();PRINTA("\n");
#else  //just show the raw value          
				PRINTA("  * Operates on channel %d\n", rf230_get_channel());
				PRINTA("  * TX Power(0=+3dBm, 15=-17.2dBm): %d\n", rf230_get_txpower());
#endif
				if (rf230_smallest_rssi) {
					PRINTA("  * Current/Last/Smallest RSSI: %d/%d/%ddBm\n", -91+(rf230_rssi()-1), -91+(rf230_last_rssi-1),-91+(rf230_smallest_rssi-1));
					rf230_smallest_rssi=0;
				} else {
					PRINTA("  * Current/Last/Smallest RSSI: %d/%d/--dBm\n", -91+(rf230_rssi()-1), -91+(rf230_last_rssi-1));
				}

				PRINTA("  * RDC Driver: %s\n", NETSTACK_CONF_RDC.name);

#if SICSLOW_ETHERNET_CONF_UPDATE_USB_ETH_STATS
				PRINTA("  * usb_eth_stats OK:tx %4lu, rx %4lu BAD:tx %2lu, rx %2lu\n",usb_eth_stat.txok,\
				usb_eth_stat.rxok,usb_eth_stat.txbad,usb_eth_stat.rxbad);
#endif
        
#if RF230_CONF_RADIOSTATS
				extern uint16_t RF230_sendpackets,RF230_receivepackets,RF230_sendfail,RF230_receivefail;
				PRINTA("  * RF230_stats   OK:tx %4d, rx %4d BAD:tx %2d, rx %2d\n",\
					RF230_sendpackets,RF230_receivepackets,RF230_sendfail,RF230_receivefail);
#endif

				PRINTA("  * Configuration: %d, USB<->ETH is ", usb_configuration_nb);
				if (usb_eth_is_active == 0) PRINTA("not ");
				PRINTA("active\n");

#if CONFIG_STACK_MONITOR
/* See contiki-raven-main.c for initialization of the magic numbers */
{
extern uint16_t __bss_end;
uint16_t p=(uint16_t)&__bss_end;
    do {
      if (*(uint16_t *)p != 0x4242) {
        PRINTA("  * Never-used stack > %d bytes\n",p-(uint16_t)&__bss_end);
        break;
      }
      p+=100;
    } while (p<RAMEND-100);
}
#endif

				break;

			case 'e':
				PRINTA("Energy Scan:\n");
				uart_usb_flush();
				{
					uint8_t i;
					uint16_t j;
					uint8_t previous_channel = rf230_get_channel();
					int8_t RSSI, maxRSSI[17];
					uint16_t accRSSI[17];
					
					bzero((void*)accRSSI,sizeof(accRSSI));
					bzero((void*)maxRSSI,sizeof(maxRSSI));
					
					for(j=0;j<(1<<12);j++) {
						for(i=11;i<=26;i++) {
							rf230_listen_channel(i);
							_delay_us(3*10);
							RSSI = rf230_rssi();  //multiplies rssi register by 3 for consistency with energy-detect register

							maxRSSI[i-11]=Max(maxRSSI[i-11],RSSI);
							accRSSI[i-11]+=RSSI;
						}
						if(j&(1<<7)) {
							Led2_on();
							if(!(j&((1<<7)-1))) {
								putc('.', stdout);
								uart_usb_flush();
							}
						}
						else
							Led2_off();
						watchdog_periodic();                   
					}

					rf230_set_channel(previous_channel);
					putc('\n', stdout);
					for(i=11;i<=26;i++) {
						uint8_t activity=Min(maxRSSI[i-11],accRSSI[i-11]/(1<<7));
						PRINTA(" %d: %02ddB ",i, -91+(maxRSSI[i-11]-1));
						for(;activity--;maxRSSI[i-11]--) {
							putc('#', stdout);
						}
						for(;maxRSSI[i-11]>0;maxRSSI[i-11]--) {
							putc(':', stdout);
						}
						putc('\n', stdout);
						uart_usb_flush();
					}
				}
				PRINTA("Done.\n");
				uart_usb_flush();
				
				break;

#if USB_CONF_BOOTLOADER
			case 'F':
				if(bootloader_is_present()) {
					PRINTA("Update firmware through USB [n]?\n");
					menustate=confirmdfuboot;
				}
				break;
#endif

#if USB_CONF_WATCHDOGRESET
			case 'R':
				{
					PRINTA("Resetting...\n");
					uart_usb_flush();
					Leds_on();
					for(i = 0; i < 10; i++)_delay_ms(100);
					Usb_detach();
					for(i = 0; i < 20; i++)_delay_ms(100);
					watchdog_reboot();
				}
				break;
#endif

#if USB_CONF_WINDOWSSWITCH
				case 'W':
				{
					PRINTA("Switching to windows mode...\n");
					uart_usb_flush();
					usb_eth_switch_to_windows_mode();
				}
				break;
#endif	
			
#if USB_CONF_STORAGE && USB_CONF_STORAGESWITCH 
			case 'U':

				//Mass storage mode
				usb_mode = mass_storage;

				//No more USB serial port
#if USB_CONF_SERIAL
				usb_stdout = NULL;
#endif
				//Deatch USB
				Usb_detach();

				//RNDIS is over
				rndis_state = 	rndis_uninitialized;

				// Reset the USB configuration
				usb_configuration_nb = 0;

				Leds_off();

				//Wait a few seconds
				for(i = 0; i < 5; i++) {
					Led0_on();
					_delay_ms(100);
					Led0_off();
					Led1_on();
					_delay_ms(100);
					Led1_off();
					Led2_on();
					_delay_ms(100);
					Led2_off();
					Led3_on();
					_delay_ms(100);
					Led3_off();
					watchdog_periodic();
				}

				Leds_off();

				//Attach USB
				Usb_attach();


				break;
#endif

			default:
				PRINTA("%c is not a valid option! h for menu\n", c);
				break;
		}


	}

	return;

}

#if !JACKDAW_CONF_ALT_LED_SCHEME
/**
    @brief This will enable the VCP_TRX_END LED for a period
*/
void vcptx_end_led(void)
{
    Led3_on();
    led3_timer = 5;
}
/** @}  */
#endif
