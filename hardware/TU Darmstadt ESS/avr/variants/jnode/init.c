#include <avr/power.h>
#include <util/delay.h>

void initVariant() {

  // switch to 16Mhz
  clock_prescale_set(clock_div_1);

  // disable JTAG to be able to use PORTF
  MCUCR = (1<<JTD);
  MCUCR = (1<<JTD);

  // reset sensors on reboot, PC5 controls sensor power
  DDRC |= (1<<5);
  _delay_ms(100);
  DDRC &= ~(1<<5);
}
