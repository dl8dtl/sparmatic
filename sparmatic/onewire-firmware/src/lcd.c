#include "lcd.h"

#include <avr/io.h>

void lcd_init()
{
	
	LCDCRB = (1<<LCDCS) | (0<<LCD2B)
	         | (1<<LCDMUX1) | (1<<LCDMUX0)
	         | (1<<LCDPM2) | (1<<LCDPM1) | (1<<LCDPM0);
	/*
	      // Das LCD wird im asynchronen mit der Frequenz
	      // des Quarzes TOSC1 = 32.768Hz als LCD Clock betrieben.
	      (1<<LCDCS)Modus (LCDCS-Bit=1)
	      // 1/3 bias is used                                               
	      |(0<<LCD2B)
	      // 1/4 Duty; COM0:3;
	      |(1<<LCDMUX1) | (1<<LCDMUX0)
	      // SEG0:24
	      |(1<<LCDPM2) | (1<<LCDPM1) | (1<<LCDPM0);
	*/
	 
	LCDFRR = (0<<LCDPS2)|(1<<LCDPS1)|(1<<LCDPS0)
	         |(0<<LCDCD2)|(0<<LCDCD1)|(1<<LCDCD0);
	/*
	      (0<<LCDPS2)|(0<<LCDPS1)|(0<<LCDPS0)    // N = 16
	      |(0<<LCDCD2)|(0<<LCDCD1)|(1<<LCDCD0);  // D = 2
	      // ergo f(frame) = 128Hz
	*/
	 
	LCDCCR = (1<<LCDDC2)|(0<<LCDDC1)|(0<<LCDDC0)
	         |(/*config.lcd_contrast*/ 10 << LCDCC0);
	/*
	      // 575 µs
	      (1<<LCDDC2)|(0<<LCDDC1)|(0<<LCDDC0)
	      // Set the initial LCD contrast level
	      |(config.lcd_contrast << LCDCC0);
	*/
	 
	LCDCRA = (1<<LCDEN)|(1<<LCDAB)|(0<<LCDIE)|(0<<LCDBL);
	/*
	      (1<<LCDEN)    // Enable LCD
	      |(1<<LCDAB)   // Low Power Waveform
	      |(0<<LCDIE)   // disable Interrupt
	      |(0<<LCDBL);  // No Blanking
	*/
}
