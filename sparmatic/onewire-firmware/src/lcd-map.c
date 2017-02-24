#include "lcd-map.h"
#include <avr/io.h>

#define set_pin(reg, regbit, data, databit) \
	{ (reg) = ((reg) & ~(1 << (regbit))) | ((!!((data) & (1 << (databit)))) << (regbit)); }
void lcd_map_top_bar(uint8_t a, uint8_t b, uint8_t c)
{
	set_pin(LCDDR11, 0, a, 0);	
	set_pin(LCDDR6,  0, a, 1);	
	set_pin(LCDDR1,  0, a, 2);	
	set_pin(LCDDR1,  1, a, 3);	
	set_pin(LCDDR6,  1, a, 4);	
	set_pin(LCDDR11, 1, a, 5);	
	
	set_pin(LCDDR11, 2, a, 6);	
	set_pin(LCDDR6,  2, a, 7);	
	set_pin(LCDDR1,  2, b, 0);	
	set_pin(LCDDR1,  3, b, 1);	
	set_pin(LCDDR6,  3, b, 2);	
	set_pin(LCDDR11, 3, b, 3);	
	
	set_pin(LCDDR11, 4, b, 4);	
	set_pin(LCDDR6,  4, b, 5);	
	set_pin(LCDDR1,  4, b, 6);	
	set_pin(LCDDR1,  5, b, 7);	
	set_pin(LCDDR6,  5, c, 0);	
	set_pin(LCDDR11, 5, c, 1);	
	
	set_pin(LCDDR16, 6, c, 2);	
	set_pin(LCDDR11, 6, c, 3);	
	set_pin(LCDDR6,  6, c, 4);	
	set_pin(LCDDR1,  6, c, 5);	
	set_pin(LCDDR1,  7, c, 6);	
	set_pin(LCDDR6,  7, c, 7);	
}
#undef set_pin
