#include <avr/power.h>
#include <avr/wdt.h>
#include <util/delay.h>

void initVariant() {

  // switch to 16Mhz
  clock_prescale_set(clock_div_1);

  // disable JTAG to be able to use PORTF
  MCUCR = (1<<JTD);
  MCUCR = (1<<JTD);

  // reset sensors on reboot, PC6 controls sensor power
  DDRC  |= (1<<6);
  PORTC |= (1<<6);
  _delay_us(500);
  PORTC &= ~(1<<6);
  DDRC  &= (1<<6);
  _delay_us(500);
}
