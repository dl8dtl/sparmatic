#include "hardware.h"

#include "motor.h"
#include "timer2.h"
#include "thermometer.h"

#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint16_t motor_position = 0;
volatile int8_t motor_direction = 0;

void motor_init()
{
	/**
	 * Initialize H-Bridge
	 */
	motor_stop();

	MOTOR_FORWARD_DDR |= (1 << MOTOR_FORWARD_BIT);
	MOTOR_BACKWARD_DDR |= (1 << MOTOR_BACKWARD_BIT);

	/**
	 * Initialize reflection light barrier sensor
	 */
	MOTOR_SENSOR_ENABLE_DDR |= (1 << MOTOR_SENSOR_ENABLE_BIT);
	MOTOR_SENSOR_ENABLE_PORT &= ~(1 << MOTOR_SENSOR_ENABLE_BIT);

	MOTOR_SENSOR_DDR &= ~(1 << MOTOR_SENSOR_BIT);

	motor_position = 0;

	/**
	 * Enable pin change interrupt for the sensor pin:
	 */
	//PCMSK0 |= (1 << PCINT3);
	//EIMSK = (1 << PCIE0);
	PCMSK0 = 0xff;
	EIMSK |= (1 << PCIE0);

}

void motor_reset_position()
{
	motor_move_backward();
	do {
		motor_position = 0xffff;
		_delay_ms(MOTOR_RESET_STEP_DURATION);
	} while(motor_position != 0xffff);
	motor_position = 0;
	motor_stop();
}

void motor_move_backward()
{
	motor_direction = -1;
	motor_sensor_enable();
	thermometer_mux_current();
	thermometer_do_conversion();
	timer2_mode_move();
	MOTOR_FORWARD_PORT &= ~(1 << MOTOR_FORWARD_BIT);
	MOTOR_BACKWARD_PORT |= (1 << MOTOR_BACKWARD_BIT);
}

void motor_move_forward()
{
	motor_direction = 1;
	motor_sensor_enable();
	thermometer_mux_current();
	thermometer_do_conversion();
	timer2_mode_move();
	MOTOR_FORWARD_PORT |= (1 << MOTOR_FORWARD_BIT);
	MOTOR_BACKWARD_PORT &= ~(1 << MOTOR_BACKWARD_BIT);
}

void motor_stop()
{
	MOTOR_FORWARD_PORT &= ~(1 << MOTOR_FORWARD_BIT);
	MOTOR_BACKWARD_PORT &= ~(1 << MOTOR_BACKWARD_BIT);
	thermometer_mux_temp();
	timer2_mode_standby();
	motor_sensor_disable();
	motor_direction = 0;
}

void motor_sensor_enable()
{
	MOTOR_SENSOR_ENABLE_PORT |= (1 << MOTOR_SENSOR_ENABLE_BIT);
}

void motor_sensor_disable()
{
	MOTOR_SENSOR_ENABLE_PORT &= ~(1 << MOTOR_SENSOR_ENABLE_BIT);
}

uint16_t motor_get_position()
{
	return motor_position;
}

uint8_t motor_get_direction()
{
	return motor_direction;
}

ISR (PCINT0_vect)
{
	if (MOTOR_SENSOR_PIN & (1 << MOTOR_SENSOR_BIT))
		motor_position += motor_direction;

	if (motor_position == 380)
		motor_stop();
}
