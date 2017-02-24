#include "timer2.h"
#include "thermometer.h"

#include <avr/interrupt.h>
#include <avr/io.h>

void timer2_init()
{
	// Enable temperature ADC mode
	timer2_mode_standby();
	
	TIMSK2 |= (1 << TOIE2);
}

void timer2_mode_standby()
{
	TCCR2A = 0;
	TCCR2A |= (1 << CS22) | (1 << CS21) | (1 << CS20);
}

void timer2_mode_move()
{
	TCCR2A = 0;
}

ISR(TIMER2_OVF_vect)
{
	thermometer_do_conversion();
}
