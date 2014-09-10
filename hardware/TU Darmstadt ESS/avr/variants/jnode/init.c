#include <avr/power.h>

void initVariant() {

  // switch to 16Mhz
  clock_prescale_set(clock_div_1);

  // disable JTAG to be able to use PORTF
  MCUCR = (1<<JTD);
  MCUCR = (1<<JTD);
}
