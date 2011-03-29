
#include "status_leds.h"

PROCESS(status_leds_process, "Status LEDs");

static uint8_t ledRX_timer, ledTX_timer, ledVCP_timer;

static enum {
	STATUS_LED_UNENUMERATED,
	STATUS_LED_INACTIVE,
	STATUS_LED_READY
} device_status_indicator;

PROCESS_THREAD(status_leds_process, ev, data_proc)
{
	static struct etimer et;
	static struct etimer et_indicator;

	PROCESS_BEGIN();

	Leds_init();

	etimer_set(&et, CLOCK_SECOND/30);
	etimer_set(&et_indicator, CLOCK_SECOND/8);

	while(1) {
		PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

		if(etimer_expired(&et_indicator)) {
			if(device_status_indicator==STATUS_LED_READY) {
				LedSTAT_on();
			} else {
				LedSTAT_toggle();
			}
			etimer_set(&et_indicator, (device_status_indicator==STATUS_LED_INACTIVE)?(CLOCK_SECOND/8):(CLOCK_SECOND/3));
		}
		
		if(etimer_expired(&et)) {
		
#if JACKDAW_CONF_DIM_ONLINE_LED&&0 //annoying 15 Hz flash
		/* Run status led at half brightness */
			if(device_status_indicator==STATUS_LED_READY) {
				LedSTAT_toggle();
				}
#endif

			if (ledRX_timer) {
				ledRX_timer--;
				if(ledRX_timer&(1<<1))
					LedRX_on();
				else
					LedRX_off();
			}
			else
				LedRX_off();

			if (ledTX_timer) {
				ledTX_timer--;
				if(ledTX_timer&(1<<1))
					LedTX_on();
				else
					LedTX_off();
			}
			else
				LedTX_off();

			if (ledVCP_timer) {
				ledVCP_timer--;
				if(ledVCP_timer&(1<<2))
					LedVCP_on();
				else
					LedVCP_off();
			}
			else
				LedVCP_off();

			etimer_set(&et, CLOCK_SECOND/30);
		}
	} // while(1)

	PROCESS_END();
}

void
status_leds_radio_tx() {
	ledTX_timer|=(1<<2);
	if(((ledTX_timer-1)&(1<<1)))
		LedTX_on();
}

void
status_leds_radio_rx() {
	ledRX_timer|=(1<<2);
	if(((ledRX_timer-1)&(1<<1)))
		LedRX_on();
}

static bool radio_is_on;

void
status_leds_radio_on() {
	if(!radio_is_on)
		status_leds_serial_tx();
	radio_is_on = true;
}

void
status_leds_radio_off() {
	radio_is_on = false;
}

void
status_leds_serial_tx() {
	ledVCP_timer|=(1<<3);
	if(((ledVCP_timer-1)&(1<<2)))
		LedVCP_on();
}

void
status_leds_serial_rx() {
	status_leds_serial_tx();
}

void
status_leds_unenumerated() {
	device_status_indicator = STATUS_LED_UNENUMERATED;
}

void
status_leds_inactive() {
	device_status_indicator = STATUS_LED_INACTIVE;
}

void
status_leds_ready() {
	device_status_indicator = STATUS_LED_READY;
}

#if JACKDAW_CONF_DIM_ONLINE_LED
void status_leds_dim() {
#if 0
	if(device_status_indicator==STATUS_LED_READY)
		LedSTAT_toggle();  //still too bright?
#else                      //use this instead
static uint8_t ledtimer;
	if(ledtimer--) {
		LedSTAT_off();
	} else if(device_status_indicator==STATUS_LED_READY) {
		LedSTAT_on();
		ledtimer=10;
	}
#endif
}
#endif
	


