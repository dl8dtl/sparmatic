;--------------------------------------
Measure_Battery_Voltage:
 ldi	Temp, 0b01011110				;AVcc as reference, internal bandgap as source
 sts	ADMUX, Temp

 nop
 nop
 nop
 ldi	Temp, 0b11000100				;ADC enable, start ADC conversion, ADC Prescaler 16
 sts	ADCSRA, Temp

_Wait_VBat_ADC1:
 lds	Temp, ADCSRA					
 sbrc	Temp, ADSC
 rjmp	_Wait_VBat_ADC1

 ldi	Temp, 0b00000100				;ADC disable, ADC Prescaler 16
 sts	ADCSRA, Temp
 
 lds	Temp, ADCL
 lds	TempH, ADCH
 sts	BatteryVoltageL, Temp
 sts	BatteryVoltageH, TempH
 
 ldi	Temp3, high(530)
 cpi	Temp, low (530)
 cpc	TempH, Temp3
 brcs	_MBV1

 ldi	Temp, LCD_Battery_SET
 ldi	TempH, $10
 call	PutSymbol
 
 lds	Temp, BatteryVoltageL
 lds	TempH, BatteryVoltageH
 ldi	Temp3, high(560)
 cpi	Temp, low (560)
 cpc	TempH, Temp3
 brcs	_MBV1
  
 rjmp	RapidShutOff 

_MBV1:
 ret



;--------------------------------------
Measure_Motor_Current:
 ldi	Temp, 0b01000010				;AVcc as reference, channel 2, motor current
 sts	ADMUX, Temp
 nop
 nop
 nop
 ldi	Temp, 0b11000100				;ADC enable, start ADC conversion, ADC Prescaler 16
 sts	ADCSRA, Temp

_Wait_MCur_ADC1:
 lds	Temp, ADCSRA					
 sbrc	Temp, ADSC
 rjmp	_Wait_MCur_ADC1

 ldi	Temp, 0b00000100				;ADC disable, ADC Prescaler 16
 sts	ADCSRA, Temp

 lds	Temp, ADCL
 lds	TempH, ADCH
 ldi	Temp3, $FF
 ldi	Temp4, $03
 sub	Temp3, Temp
 sbc	Temp4, TempH
 sts	MotorCurrent, Temp3
 ret

//End Of File//