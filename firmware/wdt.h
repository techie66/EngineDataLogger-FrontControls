/*
 * Watchdog common header
 * include this file
 * in setup() call watchdogSetup()
 * in loop call wdt_reset()
 */
#include <avr/wdt.h>

void watchdogSetup(void)
{
  cli(); // Disable all interrupts
  wdt_reset(); // reset WDT

  /*
   WDTCSR configuration:
   WDIE = 1: Interrupt Enable
   WDE = 1 :Reset Enable
   See table for time-out variations:
   WDP3 = 0 :For 1000ms Time-out
   WDP2 = 1 :For 1000ms Time-out
   WDP1 = 1 :For 1000ms Time-out
   WDP0 = 0 :For 1000ms Time-out
  */
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings:
   WDTCSR = (1<<WDIE) | (1<<WDE) |
  (0<<WDP3) | (1<<WDP2) | (1<<WDP1) |
  (0<<WDP0);

  sei();
}

void watchdogDisable(void)
{
  MCUSR = MCUSR & B11110111;
  wdt_disable();
}


ISR(WDT_vect) // Watchdog timer interrupt.
{
  // Include your code here - be careful not to use functions they may cause the interrupt to hang and
  // prevent a reset.
}


