/**
 * \file
 *   This files provides functions to control the 74HC595D chip on the
 *   Sensinode N740s. This is an 8-bit serial in-parallel out shift register.
 *
 *   LEDs are connected to this chip. It also serves other functions such as
 *   enabling/disabling the Accelerometer (see n740-ser-par.h).
 *
 *   We can:
 *     - write a new instruction
 *     - remember and retrieve the last instruction sent
 *
 *   The chip is connected to CPU pins as follows:
 *   - P0_2: Serial Data Input
 *   - P1_3: Shift Register Clock Input
 *   - P1_1: Storage Register Clock
 *
 *   This file can be placed in any bank.
 *
 * \author
 *         George Oikonomou - <oikonomou@users.sourceforge.net>
 */

#include "dev/banked.h"
#include "dev/sensinode-sensors.h"
#include "dev/n740-ser-par.h"

/*
 * This variable stores the most recent instruction sent to the ser-par chip.
 * We declare it as static and return its value through n740_ser_par_get().
 */
static uint8_t ser_par_status;

/*---------------------------------------------------------------------------*/
/* Init the serial-parallel chip:
 *   - Set I/O direction for all 3 pins (P0_2, P1_1 and P1_3) to output
 *   - We start with value 0x02 (Flash Enable / Chip Select bit set)
 */
void
n740_ser_par_init() __banked
{
  /* bus_init and uart1_init also touch the I/O direction for those pins */
  uint8_t new_status;
  P1DIR |= 0x0A;
  P0DIR |= 0x04;

  n740_ser_par_set(new_status);
}

/*---------------------------------------------------------------------------*/
/*
 * Send a command to the N740 serial-parallel chip. Each command is a single
 * byte, each bit controls a different feature on the sensor.
 */
void
n740_ser_par_set(uint8_t data) __banked
{
  uint8_t i;
  uint8_t mask = 1;
  uint8_t temp = 0;

  ENTER_CRITICAL();
  /* bit-by-bit */
  for(i = 0; i < 8; i++) {
    temp = (data & mask);
    /* Is the bit set? */
    if(i && temp) {
      /* If it was set, we want to send 1 */
      temp >>= i;
    }
    /* Send the bit */
    P0_2 = temp;
    /* Shift */
    P1_3 = 1;
    P1_3 = 0;
    mask <<= 1;
  }
  /* Move to Par-Out */
  P1_1 = 1;
  P1_1 = 0;
  EXIT_CRITICAL();

  /* Right, we're done. Save the new status in ser_par_status */
  ser_par_status = data;
}

/*---------------------------------------------------------------------------*/
/* This function returns the last value sent to the ser-par chip on the N740.
 *
 * The caveat here is that we must always use n740_set_ser_par() to send
 * commands to the ser-par chip, never write directly.
 *
 * If any other function sends a command directly, ser_par_status and the
 * actual status will end up out of sync.
 */
uint8_t
n740_ser_par_get() __banked
{
  return ser_par_status;
}
