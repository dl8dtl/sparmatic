#include "thermometer.h"
#include "hardware.h"
#include "motor.h"

#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint16_t temperature = 0;
volatile uint8_t mux_pos = 0;

void thermometer_init()
{
	// Set ADC clock prescaler to 1/32
	ADCSRA |= (1 << ADPS2) | (0 << ADPS1) | (1 << ADPS0);

	// Set VREF to Vcc
	ADMUX |= (0 << REFS1) | (1 << REFS0);	

	thermometer_mux_temp();

	// Enable ADC interrupts
	ADCSRA |= (1 << ADIE);
	
	THERMOMETER_ENABLE_PORT &= ~(1 << THERMOMETER_ENABLE_BIT);
	THERMOMETER_ENABLE_DDR |= (1 << THERMOMETER_ENABLE_BIT);
}

void thermometer_do_conversion()
{
	THERMOMETER_ENABLE_PORT |= (1 << THERMOMETER_ENABLE_BIT);
	
	// enable adc
	ADCSRA |= (1 << ADEN);

	// start conversion
	ADCSRA |= (1 << ADSC);

	// Not required: We are using interrupts.
	//while(!(ADCSRA & (1 << ADSC)));
	//temperature = ADCL;
	//temperature |= (ADCH << 8);
}

void thermometer_mux_temp()
{
	mux_pos = 0;
	ADMUX &= ~((1 << MUX4) | 
		 (1 << MUX3) | 
		 (1 << MUX2) | 
		 (1 << MUX1) | 
		 (1 << MUX0));
	ADMUX |= (0 << MUX4) | 
		 (0 << MUX3) | 
		 (0 << MUX2) | 
		 (0 << MUX1) | 
		 (1 << MUX0);
}

void thermometer_mux_current()
{
	mux_pos = 1;
	ADMUX &= ~((1 << MUX4) | 
		 (1 << MUX3) | 
		 (1 << MUX2) | 
		 (1 << MUX1) | 
		 (1 << MUX0));
	ADMUX |= (0 << MUX4) | 
		 (0 << MUX3) | 
		 (0 << MUX2) | 
		 (1 << MUX1) | 
		 (0 << MUX0);
}

void thermometer_sleep()
{
	THERMOMETER_ENABLE_PORT &= ~(1 << THERMOMETER_ENABLE_BIT);

	// disable adc
	ADCSRA &= ~(1 << ADEN);
}

uint16_t thermometer_get_temperature()
{
	// convert to whatever jeff wants
	return temperature;
}

ISR(ADC_vect)
{
	uint16_t adcval;
	adcval = ADCL;
	adcval |= (ADCH << 8);
	if (motor_get_direction())
	{
		if (adcval < MOTOR_CURRENT_LIMIT)
		{
			motor_stop();
			// @todo error flag
		}
		ADCSRA |= (1 << ADSC);
	}
	else
	{
		temperature = adcval;
		thermometer_sleep();
	}
}
