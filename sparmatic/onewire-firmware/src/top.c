#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "motor.h"
#include "hardware.h"
#include "lcd.h"
#include "lcd-map.h"
#include "thermometer.h"
#include "timer2.h"

int main()
{
	uint32_t ctr = 0;

	/**
	 * Initialize hardware
	 */
	motor_init();
	lcd_init();
	thermometer_init();
	timer2_init();

	LCDDR0 = 0x01;

	sei();
	_delay_ms(500);

	/**
	 * Move valve into reset position
	 */
	motor_reset_position();


	/**
	 * @todo: close valve again
	 */
	motor_move_forward();
	_delay_ms(1000);
	
	while(1)
	{
		uint16_t temp;
		temp = thermometer_get_temperature();
		lcd_map_top_bar(temp, temp >> 8, 0xff);
		_delay_ms(100);
	}
	//ctr = 0;
	//while(1)
	//{
	//	if (!ctr) ctr = 1;
	//	lcd_map_top_bar(ctr, ctr >> 8, ctr >> 16);
	//	ctr <<= 1;
	//	_delay_ms(1000);
	//}

	while(1)
	{
#define r EIFR & (1 << 6)

		motor_move_forward();
		for (ctr = 0; ctr < 50; ctr++)
		{
			_delay_ms(50);
			lcd_map_top_bar(motor_get_position(), motor_get_position() >> 8, !!(r));
		}
		motor_stop();
		_delay_ms(500);
		motor_move_backward();
		for (ctr = 0; ctr < 60; ctr++)
		{
			_delay_ms(50);
			lcd_map_top_bar(motor_get_position(), motor_get_position() >> 8, !!(r));
		}
		motor_stop();
		_delay_ms(500);
	}

//	lcd_map_top_bar(0xff, 0xff, 0);

//	while (1)
//	{
//		ctr ++;
//		lcd_map_top_bar(ctr, ctr >> 8, ctr >> 16);
//		_delay_ms(1);
//	}
//	while(1)
//	{
//		motor_move_forward();
//		_delay_ms(500);
//		motor_stop();
//		_delay_ms(500);
//		motor_move_backward();
//		_delay_ms(500);
//		motor_stop();
//		_delay_ms(500);
//	}

	while(1);
}
