#ifndef UART_H
#define UART_H

#include "contiki-conf.h"

#include "cc2430_sfr.h"

#ifdef UART_ZERO_CONF_ENABLE
#define UART_ZERO_ENABLE UART_ZERO_CONF_ENABLE
#else
#define UART_ZERO_ENABLE 0
#endif

#ifdef UART_ONE_CONF_ENABLE
#define UART_ONE_ENABLE UART_ONE_CONF_ENABLE
#else
#define UART_ONE_ENABLE 0
#endif

#if UART_ZERO_ENABLE
void uart0_init(uint32_t speed) __banked;
void uart0_writeb(uint8_t byte);

void uart0_set_input(int (*input)(unsigned char c));

void uart0_rxISR( void ) __interrupt (URX0_VECTOR);
void uart0_txISR( void ) __interrupt (UTX0_VECTOR);
#endif

#if UART_ONE_ENABLE
void uart1_init(uint32_t speed) __banked;
void uart1_writeb(uint8_t byte);

void uart1_set_input(int (*input)(unsigned char c));

void uart1_rxISR( void ) __interrupt (URX1_VECTOR);
void uart1_txISR( void ) __interrupt (UTX1_VECTOR);
#else
#define uart1_init(...)
#define uart1_writeb(...)
#define uart1_set_input(...)
#endif

#endif /*UART_H*/
