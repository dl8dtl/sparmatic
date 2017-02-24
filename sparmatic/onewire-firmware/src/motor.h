#ifndef _MOTOR_H
#define _MOTOR_H

#include <inttypes.h>

void motor_init();
void motor_move_backward();
void motor_move_forward();
void motor_stop();
void motor_sensor_enable();
void motor_sensor_disable();
void motor_reset_position();
uint16_t motor_get_position();
uint8_t motor_get_direction();

#endif /* #ifndef _MOTOR_H */
