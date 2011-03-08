#ifndef UART_H
#define UART_H

#include "contiki-conf.h"

#include "cc2430_sfr.h"
#include "8051def.h"

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

void uart0_rx_ISR( void ) __interrupt (URX0_VECTOR);
void uart0_tx_ISR( void ) __interrupt (UTX0_VECTOR);
/* Macro to turn on / off UART RX Interrupt */
#define UART0_RX_INT(v) IEN0_URX0IE = v
#endif

#if UART_ONE_ENABLE
void uart1_init(uint32_t speed) __banked;
void uart1_writeb(uint8_t byte);

void uart1_set_input(int (*input)(unsigned char c));
#if UART_ONE_CONF_WITH_INPUT
void uart1_rx_ISR( void ) __interrupt (URX1_VECTOR);
void uart1_tx_ISR( void ) __interrupt (UTX1_VECTOR);
/* Macro to turn on / off UART RX Interrupt */
#define UART1_RX_INT(v) IEN0_URX1IE = v
#else
#define UART1_RX_INT(v)
#endif /* UART_ONE_CONF_WITH_INPUT */
#else
#define uart1_init(...)
#define uart1_writeb(...)
#define uart1_set_input(...)
#define UART1_RX_INT(v)
#endif /* UART_ONE_ENABLE */

#endif /*UART_H*/
