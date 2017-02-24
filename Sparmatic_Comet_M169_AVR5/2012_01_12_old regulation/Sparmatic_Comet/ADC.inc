Measure_Motor_Current:
 sbis	Status0, _Adapt
 ret
 
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