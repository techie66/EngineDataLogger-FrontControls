/*
 * Watchdog common header
 * include this file
 * in setup() call watchdogSetup()
 * in loop call wdt_reset()
 */

/*
WDTO_15MS 0
WDTO_30MS 1
WDTO_60MS 2
WDTO_120MS 3
WDTO_250MS 4
WDTO_500MS 5
WDTO_1S 6
WDTO_2S 7
WDTO_4S 8
WDTO_8S 9
*/

#include <avr/wdt.h>

void watchdogSetup(void)
{
  cli(); // Disable all interrupts
 /*
   WDTCSR configuration:
   WDIE = 1: Interrupt Enable
   WDE = 1 :Reset Enable
 */ 
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings:
   WDTCSR = (1<<WDE) |
     (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);

  wdt_reset(); // reset WDT
  MCUSR = MCUSR & B11110111; // Clear the reset flag, the WDRF bit (bit 3) of MCUSR.
  sei();
}

void watchdogDisable(void)
{
  MCUSR = MCUSR & B11110111;
  wdt_disable();
}


//ISR(WDT_vect) // Watchdog timer interrupt.
//{
  // Include your code here - be careful not to use functions they may cause the interrupt to hang and
  // prevent a reset.
//}


