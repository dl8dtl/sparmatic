#ifndef _THERMOMETER_H
#define _THERMOMETER_H

#include <inttypes.h>

void thermometer_init();
void thermometer_do_conversion();
uint16_t thermometer_get_temperature();
void thermometer_mux_temp();
void thermometer_mux_current();

#endif /* #ifndef _THERMOMETER_H */
