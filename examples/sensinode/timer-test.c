/**
 * \file
 *         Tests related to clocks and timers
 *
 *         This is clock_test.c plus a small addition by George Oikonomou
 *         (Loughborough University)in order to test the rtimer
 *
 * \author
 *         Zach Shelby <zach@sensinode.com>
 */

#include "contiki.h"
#include "sys/clock.h"
#include "dev/leds.h"
#include <stdio.h> /* For printf() */

/*---------------------------------------------------------------------------*/
PROCESS(clock_test_process, "Clock test process");
AUTOSTART_PROCESSES(&clock_test_process);
/*---------------------------------------------------------------------------*/
void
rt_callback(struct rtimer *t, void *ptr) {
  printf("Task called at %u\n", RTIMER_NOW());
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(clock_test_process, ev, data)
{
  static struct etimer et;
  static struct rtimer rt;

  static clock_time_t count, start_count, end_count, diff;
  static unsigned long sec;
  static u8_t i;
  uint16_t rt_now, rt_for;

  PROCESS_BEGIN();
  rtimer_init();
  printf("Clock delay test (10 x (10,000xi) cycles):\n");
  i = 1;
  while (i < 6) {
    start_count = clock_time();
    clock_delay(10000 * i);
    end_count = clock_time();
    diff = end_count - start_count;
    printf("Delayed %u = %u ticks = ~%u ms\n", 10000 * i, diff, diff * 8);
    i++;
  }
  
  printf("Rtimer Test (10 x 1s):\n");
  i = 0;
  while(i < 10) {
    etimer_set(&et, 2*CLOCK_SECOND);
    puts("=======================");
    rt_now = RTIMER_NOW();
    rt_for = rt_now + RTIMER_SECOND;
    printf("%Now=%u - For=%u\n", rt_now, rt_for);
    if (rtimer_set(&rt, rt_for, 1,
              (void (*)(struct rtimer *, void *))rt_callback, NULL) != RTIMER_OK) {
      printf("Error setting\n");
    }

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    i++;
  }

  printf("Clock tick and etimer test (10 x 1s):\n");
  i = 0;
  while(i < 10) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);

    count = clock_time();
    printf("%u ticks\n", count);

    leds_toggle(LEDS_RED);
    i++;
  }

  printf("Clock seconds test (10 x 5s):\n");
  i = 0;
  while(i < 10) {
    etimer_set(&et, 5 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);

    sec = clock_seconds();
    printf("%u seconds\n", (u16_t) sec);

    leds_toggle(LEDS_GREEN);
    i++;
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
