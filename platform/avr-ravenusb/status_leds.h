

#ifndef __STATUS_LEDS_H__
#define __STATUS_LEDS_H__

#include "contiki.h"
#include "config.h"

PROCESS_NAME(status_leds_process);

void status_leds_radio_tx();
void status_leds_radio_rx();
void status_leds_radio_on();
void status_leds_radio_off();
void status_leds_serial_tx();
void status_leds_serial_rx();

void status_leds_unenumerated();
void status_leds_inactive();
void status_leds_ready();
void status_leds_dim();

#endif // __STATUS_LEDS_H__