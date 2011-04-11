#ifndef UART_H
#define UART_H

#include "contiki-conf.h"

#include "cc2430_sfr.h"
#include "8051def.h"

/*---------------------------------------------------------------------------*/
/* UART0/1 Enable - Disable */
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
/*---------------------------------------------------------------------------*/
/* UART0 Function Declarations */
#if UART_ZERO_ENABLE
void uart0_init();
void uart0_writeb(uint8_t byte);

void uart0_set_input(int (*input)(unsigned char c));

void uart0_rx_ISR( void ) __interrupt (URX0_VECTOR);
void uart0_tx_ISR( void ) __interrupt (UTX0_VECTOR);
/* Macro to turn on / off UART RX Interrupt */
#define UART0_RX_INT(v) IEN0_URX0IE = v
#endif
/*---------------------------------------------------------------------------*/
/* UART1 Function Declarations */
#if UART_ONE_ENABLE
void uart1_init();
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
/*---------------------------------------------------------------------------*/
/* UART BAUD Rates */
/*
 * Macro to set speed of UART N by setting the UnBAUD SFR to M and the
 * UnGCR SRF to E. See the cc2430 datasheet for possible values of M and E
 */
#define UART_SET_SPEED(N, M, E) do{ U##N##BAUD = M; U##N##GCR = E; } while(0)

/*
 * Sample Values for M and E in the macro above to achieve some common BAUD
 * rates. For more values, see the cc2430 datasheet
 */
/* 2000000 - cc2430 theoretical MAX when using the 32MHz clock */
#define UART_2K_M      0
#define UART_2K_E     16
/* 1000000 - cc2430 theoretical MAX when using the 16MHz clock */
#define UART_1K_M      0
#define UART_1K_E     15
/* 921600 */
#define UART_921_M   216
#define UART_921_E    14
/* 460800 Higher values lead to problems when the node needs to RX */
#define UART_460_M   216
#define UART_460_E    13
/* 115200 */
#define UART_115_M   216
#define UART_115_E    11
/* 38400 */
#define UART_38_M     59
#define UART_38_E     10
/* 9600 */
#define UART_9_M      59
#define UART_9_E       8

#endif /*UART_H*/
