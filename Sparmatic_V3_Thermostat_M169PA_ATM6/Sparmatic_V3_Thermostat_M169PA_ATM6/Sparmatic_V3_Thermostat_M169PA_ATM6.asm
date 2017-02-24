/*
 * Sparmatic_V3_Thermostat_M169PA_ATM6.asm
 *
 *  Created: 20.01.2015 20:07:00
 *   Author: TrR_KB
 */ 


.include "Def_Equ.inc"					;definition file
.include "Init.inc"						;initialisation file

 rcall	StartMain						;sound


MainLoop:
 rcall	Adaptation
 
 sbis	Status0, _Adapt					;don´t execute in adaptation mode
 rcall	Show_Current_Temperature
  
 sbis	Status0, _Adapt					;don´t execute in adaptation mode
 rcall	Show_Current_Time
 
 sbis	Status0, _Adapt					;don´t execute in adaptation mode
 call	User_Action
 
 sbis	Status0, _Adapt					;don´t execute in adaptation mode
 rcall	Timer2Mode

 sbis	Status0, _Adapt					;don´t execute in adaptation mode
 rcall	Mode2SetTemperature
 
 sbis	Status0, _Adapt					;don´t execute in adaptation mode
 rcall	Calc_Temperature

 sbi	PORTE, 5
 call	MotorControl
 call	Clock
 rcall	Copy_DisplayBuffers 

 sbic	PinE, 5									;BatteryTestPin
 jmp	RapidShutOff							;if set, immediately save clock and go to sleep 
 cbi	PORTE, 5

 lds	Temp, UserSettings1
 sbrc	Temp, _ExternalTempSensor
 rcall	IO_Interface					


 GoSleep:
 in		Temp, Status2						;don´t sleep while communicating externally
 sbrc	Temp, _IO_Running
 rjmp	MainLoop
  
 cpi	ActiveCT, 0							;do not disable pullups in active mode
 brne	_GS1  
 
 in		Temp, PinB							;read button state before entering sleep
 andi	Temp, 0b10000001					;any rotary pin that is connected to GND, disable corresponding pullup to save power (~150µA worst case)
 ori	Temp, 0b01111100					;leave button and SPI pullups on
 out	PortB, Temp						
  
_GS1:
 ;cbi	DebugPort, Debug1
 ldi	Temp, (1<<SM1) | (1<<SM0) | (1<<SE)	;Power save sleep mode
 out	SMCR, Temp

 sleep
 
 sbi	PortB, 7							;re-enable rotary pullups
 sbi	PortB, 0
  
 ldi	Temp, (1<<SM1) | (1<<SM0) | (0<<SE)	;disable sleep
 out	SMCR, Temp
 ;sbi	DebugPort, Debug1

 rjmp	MainLoop



;----------------------------------------------
IO_Interface:
 in		Temp, Status1						;if one data byte is received, execute routine
 sbrs	Temp, _IO_Receive
 ret
 
 in		Temp, Status2						;if routine is called before starting the interface, exit
 sbrs	Temp, _IO_Running
 rjmp	_IO_Off

 
_IO_GetData:
 ldi	XL, low(ExternalDataBuffer)
 ldi	XH, high(ExternalDataBuffer)
 lds	TempH, IO_ByteCT						
 add	XL, TempH
 adc	XH, Null

 lds	Temp, IO_DataBuffer
 st		X, Temp
 inc	TempH
 sts	IO_ByteCT, TempH
 cpi	TempH, 32						
 breq	_IO_FrameComplete					;if data is completely read, end routine


_IO_Off:
 cli
 in		Temp, Status1
 cbr	Temp, (1<<_IO_Receive)				;clear IO receive flag
 out	Status1, Temp
 sei 
 ret

 
_IO_FrameComplete:
 in		Temp, Status2
 cbr	Temp, (1<<_IO_Running)				;stop interface
 out	Status2, Temp

 ldi	Temp, 0b11110001					;disable IO_clock as pin change interrupt
 sts	PCMSK1, Temp
 
 lds	Temp, ExternalDataBuffer			;process received data: filter packet type
 cpi	Temp, 0x01
 breq	_IO_DataPacket
 rjmp	_IO_End


_IO_DataPacket:  
 lds	Temp, ExternalDataBuffer+1			;store received data as temperature value
 sts	TempIntL, Temp
 lds	Temp, ExternalDataBuffer+2
 sts	TempIntH, Temp
 

_IO_End:
 lds	Temp, DisplayBuffer1+2				;clear tower symbol
 andi	Temp, 0b01111111
 sts	DisplayBuffer1+2, Temp
 lds	Temp, DisplayBuffer2+2					
 andi	Temp, 0b01111111
 sts	DisplayBuffer2+2, Temp
 
 ldi	Temp, 238
 sts	Radio_RXTimeslot, Temp
 rjmp	_IO_Off



;---------------
START_IO:
 sbi	IO_DDR, IO_SCK						;send WakeUp pulse to slave: set clock pin to output
 nop
 cbi	IO_PORT, IO_SCK						;transmit low pulse
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 sbi	IO_PORT, IO_SCK
 nop
 nop
 cbi	IO_DDR, IO_SCK	 					;set clock pin to input, leave pullup on
 
 in		Temp, Status2
 sbr	Temp, (1<<_IO_Running)				;start interface
 out	Status2, Temp
 
 sts	IO_BitCT, Null
 sts	IO_ByteCT, Null

 ldi	Temp, 0b11111001					;enable IO_clock as pin change interrupt
 sts	PCMSK1, Temp
 
 lds	Temp, DisplayBuffer1+2				;set tower symbol
 ori	Temp, 0b10000000
 sts	DisplayBuffer1+2, Temp
 lds	Temp, DisplayBuffer2+2					
 ori	Temp, 0b10000000
 sts	DisplayBuffer2+2, Temp
 ret


  
;-------------------------------------------
Timer2Mode:
 in		Temp, Status1
 sbrc	Temp, _AutoManu							;no timer check in manual mode 
 ret

 sbic	Status0, _MenuOn						;no timer check if a menu is active
 ret

 in		Temp, Status2
 sbrc	Temp, _FastModeScan						;do timer compares immediately, if fastmodescan flag is set
 rjmp	_T2SetTemp0
 
 lds	Temp, Seconds							;do timer compares only on first of ten Minutes and on first second
 cpi	Temp, 0
 breq	_T2SetTemp
 ret
 
_T2SetTemp:
 lds	Temp, Minutes
 rcall	DivDez
 ldi	Temp, '0'
 cp 	Di0, Temp
 breq	_T2SetTemp0
 ret
 
_T2SetTemp0:
 ldi	XL, low(DailyTimer)
 ldi	XH, high(DailyTimer)
 lds	Temp, WDays

 lds	TempH, UserSettings1
 sbrs	TempH, _Holiday
 rjmp	_T2SetTemp01
 ldi	Temp, BlockHoliday1
 sbrs	TempH, _2ndHoliday
 rjmp	_T2SetTemp01
 ldi	Temp, BlockHoliday2
  
_T2SetTemp01:
 ldi	Temp3, 9							;number of timers per Day
 mul	Temp, Temp3
 add	XL, r0
 adc	XH, r1
 
 lds	TempH, Hours
 lds	Temp, Minutes
 rcall	HourTenMin2TimerValue

 ld 	Temp,  X+
 cpi	Temp, 255							;if timer is inactive, skip	compare
 breq	_T2SetTemp1
 cp		Di0, Temp
 brcs	_T2SetTemp1
 ldi	TemperatureMode, InHouseMode

_T2SetTemp1:
 ld 	Temp,  X+
 cpi	Temp, 255							;if timer is inactive, skip	compare
 breq	_T2SetTemp2
 cp		Di0, Temp
 brcs	_T2SetTemp2
 ldi	TemperatureMode, OffHouseMode
 
_T2SetTemp2:
 ld 	Temp,  X+
 cpi	Temp, 255							;if timer is inactive, skip	compare
 breq	_T2SetTemp3
 cp		Di0, Temp
 brcs	_T2SetTemp3
 ldi	TemperatureMode, InHouseMode

_T2SetTemp3:
 ld 	Temp,  X+
 cpi	Temp, 255							;if timer is inactive, skip	compare
 breq	_T2SetTemp4
 cp		Di0, Temp
 brcs	_T2SetTemp4
 ldi	TemperatureMode, OffHouseMode

_T2SetTemp4:
 ld 	Temp,  X+
 cpi	Temp, 255							;if timer is inactive, skip	compare
 breq	_T2SetTemp5
 cp		Di0, Temp
 brcs	_T2SetTemp5
 ldi	TemperatureMode, InHouseMode

_T2SetTemp5:
 ld 	Temp,  X+
 cpi	Temp, 255							;if timer is inactive, skip	compare
 breq	_T2SetTemp6
 cp		Di0, Temp
 brcs	_T2SetTemp6
 ldi	TemperatureMode, OffHouseMode

_T2SetTemp6:
 ld 	Temp,  X+
 cpi	Temp, 255							;if timer is inactive, skip	compare
 breq	_T2SetTemp7
 cp		Di0, Temp
 brcs	_T2SetTemp7
 ldi	TemperatureMode, InHouseMode

_T2SetTemp7:
 ld 	Temp,  X+
 cpi	Temp, 255							;if timer is inactive, skip	compare
 breq	_T2SetTemp8
 cp		Di0, Temp
 brcs	_T2SetTemp8
 ldi	TemperatureMode, OffHouseMode

_T2SetTemp8:
 ld 	Temp,  X+
 cpi	Temp, 255							;if timer is inactive, skip	compare
 breq	_T2SetTemp9
 cp		Di0, Temp
 brcs	_T2SetTemp9
 ldi	TemperatureMode, NightMode
  
_T2SetTemp9:
 in		Temp, Status2
 sbr 	Temp, (1<<_NewTempMode)
 cbr 	Temp, (1<<_FastModeScan)
 out	Status2, Temp
 ret



;--------------------------------------------------
Mode2SetTemperature:
 in		Temp, Status2
 sbrs	Temp, _NewTempMode
 ret

 cbr	Temp, (1<<_NewTempMode)
 out	Status2, Temp

 cpi	TemperatureMode, NoTempMode
 breq	_NoTempMode
 cpi	TemperatureMode, InHouseMode
 breq	_InHouseMode 
 cpi	TemperatureMode, OffHouseMode
 breq	_OffHouseMode
 cpi	TemperatureMode, NightMode
 breq	_NightMode
 
_NoTempMode:
 rcall	ClearInHouse
 rcall	ClearOffHouse
 rcall	ClearMoon
 ret

_InHouseMode:
 rcall	ClearOffHouse
 rcall	ClearMoon
 rcall	SetInHouse
 lds	Di0, InHouseTempL
 lds	Di1, InHouseTempH 
 rcall	_TempModeChanged
 lds	Temp, SetTempL
 lds	TempH, SetTempH
 cp		Temp, Di0
 cpc	TempH, Di1
 breq	_M2ST_End
 rcall	BlinkInHouse
 rjmp	_M2ST_End

_OffHouseMode:
 rcall	ClearInHouse
 rcall	ClearMoon
 rcall	SetOffHouse
 lds	Di0, OffHouseTempL
 lds	Di1, OffHouseTempH
 rcall	_TempModeChanged
 lds	Temp, SetTempL
 lds	TempH, SetTempH
 cp		Temp, Di0
 cpc	TempH, Di1
 breq	_M2ST_End
 rcall	BlinkOffHouse
 rjmp	_M2ST_End

_NightMode:
 rcall	ClearOffHouse
 rcall	ClearInHouse
 rcall	SetMoon
 lds	Di0, NightTempL
 lds	Di1, NightTempH
 rcall	_TempModeChanged
 lds	Temp, SetTempL
 lds	TempH, SetTempH
 cp		Temp, Di0
 cpc	TempH, Di1
 breq	_M2ST_End
 rcall	BlinkMoon

_M2ST_End:
 ret

_TempModeChanged:										;alternate set temp only automatically, if temperature mode just toggles
 lds	Temp, TempModeOld
 sts	TempModeOld, TemperatureMode
 cp		TemperatureMode, Temp
 brne	_M2ST_E1
 ret

_M2ST_E1:
 sts	SetTempL, Di0
 sts	SetTempH, Di1
 ret


;-------------------------------------------
Store_Time:
 ldi	XL, low(Minutes)
 ldi	XH, high(Minutes)

 ldi	TempH, low(Minutes_EE)
 ldi	Temp3, high(Minutes_EE)
 ldi	Temp4, 5

_Store_Time_Loop:
 ld		Temp, X+
 rcall	EEWrite
 inc	TempH
 dec	Temp4
 brne	_Store_Time_Loop
 ret

;-------------------------------------------
ReadBack_Time:
 ldi	XL, low(Minutes)
 ldi	XH, high(Minutes)

 ldi	TempH, low(Minutes_EE)
 ldi	Temp3, high(Minutes_EE)
 ldi	Temp4, 5

_RB_Time_Loop:
 rcall	EERead
 st		X+, Temp
 inc	TempH
 dec	Temp4
 brne	_RB_Time_Loop
 ret


;-------------------------------------
StoreTimers2EEPROM:
 ldi	XL, low(DailyTimer)
 ldi	XH, high(DailyTimer)
 ldi	TempH, low(EE_DailyTimer)
 ldi	Temp3, high(EE_DailyTimer)
 ldi	Temp4, 108
 
_Timer_EEStoreLoop:
 ld		Di0, X+
 rcall	EERead
 cp		Di0, Temp
 breq	_T_EESTL1
 mov	Temp, Di0
 rcall	EEWrite
 
_T_EESTL1: 
 inc	TempH
 dec	Temp4
 brne	_Timer_EEStoreLoop
 ret


;-------------------------------------
ReadEEPROM2Timers:
 ldi	XL, low(DailyTimer)
 ldi	XH, high(DailyTimer)
 ldi	TempH, low(EE_DailyTimer)
 ldi	Temp3, high(EE_DailyTimer)
 ldi	Temp4, 108
 
_R_EE2TimerLoop:
 rcall	EERead
 st		X+, Temp
 inc	TempH
 dec	Temp4
 brne	_R_EE2TimerLoop
 ret


;-------------------------------------
StoreTemperatures2EEPROM:
 ldi	XL, low(InHouseTempL)
 ldi	XH, high(InHouseTempL)
 ldi	TempH, low(EE_Temperatures)
 ldi	Temp3, high(EE_Temperatures)
 ldi	Temp4, 6
 
_Temperature_EEStoreLoop:
 ld		Di0, X+
 rcall	EERead
 cp		Di0, Temp
 breq	_Temp_EESTL1
 mov	Temp, Di0
 rcall	EEWrite
 
_Temp_EESTL1: 
 inc	TempH
 dec	Temp4
 brne	_Temperature_EEStoreLoop
 ret


;-------------------------------------
ReadEEPROM2Temperatures:
 ldi	XL, low(InHouseTempL)
 ldi	XH, high(InHouseTempL)
 ldi	TempH, low(EE_Temperatures)
 ldi	Temp3, high(EE_Temperatures)
 ldi	Temp4, 6
 
_R_EE2TemperatureLoop:
 rcall	EERead
 st		X+, Temp
 inc	TempH
 dec	Temp4
 brne	_R_EE2TemperatureLoop
 ret





;-------------------------------------------
 StartMain:
 ldi	Temp, 100				;Frequency
 out	OCR0A, Temp

 ldi	Temp, 255				;Length
 sts	BeepLen, Temp
 sts	BeepCT, Null

 in		Temp, TIFR0				;clear interrupt flag
 sbr	Temp, (1<<OCF0A)
 out	TIFR0, Temp
 
 lds	Temp, TIMSK0
 sbr	Temp, (1<<OCIE0A)
 sts	TIMSK0, Temp
 
_WaitBeep:
 lds	Temp, TIMSK0
 sbrc	Temp, OCIE0A
 rjmp	_WaitBeep

;start Adaptation 
 ser	Temp
 sts	FreeMotorCurrent, Temp
 sts	PositionL, Null
 sts	PositionH, Null
 /*ldi	Temp, low(MaxAdaptWay)
 sts	OutLimitL, Temp
 sts	ValveBOTL, Temp
 ldi	Temp, high(MaxAdaptWay)
 sts	OutLimitH, Temp
 sts	ValveBOTH, Temp*/
 sts	AdaptStep, Null
 sbi	Status0, _Adapt
 ret



;----------------------------------------------------------------------------------------
;Calculate temperatures from internal NTC voltage divider using the formula: R2=R1*U2/U1
;----------------------------------------------------------------------------------------
Calc_Temperature:
 in 	Temp, Status1
 sbrs	Temp, _NewTemp
 ret
 
 sbi	Internal_NTC_DDR, Internal_NTC_On	
 sbi	Internal_NTC_Port, Internal_NTC_On	;switch NTC voltage divider on
 ldi	Temp, 0b01000011					;AVcc as reference, channel 3, internal NTC 
 sts	ADMUX, Temp
 nop
 nop
 nop
 ldi	Temp, 0b11000100					;ADC enable, start ADC conversion, ADC Prescaler 16
 sts	ADCSRA, Temp

_Wait_NTC_ADC1:
 lds	Temp, ADCSRA						;measure upper voltage on voltage divider (U1)
 sbrc	Temp, ADSC
 rjmp	_Wait_NTC_ADC1

 lds	Temp, ADCL
 lds	TempH, ADCH
 sts	iNTCVL, Temp						;store U1
 sts	iNTCVH, TempH

 ldi	Temp, 0b01000001					;AVcc as reference, channel3, internal NTC 
 sts	ADMUX, Temp
 nop
 nop
 nop
 ldi	Temp, 0b11000100					;ADC enable, start ADC conversion, ADC Prescaler 16
 sts	ADCSRA, Temp

_Wait_NTC_ADC2:
 lds	Temp, ADCSRA						;measure NTC voltage
 sbrc	Temp, ADSC	
 rjmp	_Wait_NTC_ADC2


_CalcT1:
 cli
 in 	Temp, Status1
 cbr	Temp, (1<<_NewTemp)
 out 	Status1, Temp
 sei

 cbi	Internal_NTC_DDR, Internal_NTC_On
 cbi	Internal_NTC_Port, Internal_NTC_On	;switch NTC voltage divider off
 ldi	Temp, 0b00000100					;ADC disable, ADC Prescaler 16
 sts	ADCSRA, Temp

 lds	Temp, ADCL
 lds	TempH, ADCH
 sts	NTCIntL, Temp						;store NTC voltage
 sts	NTCIntH, TempH


 lds	Temp3, UserSettings1
 sbrs	Temp3, _ExternalTempSensor
 rjmp	_CalcT11

 ;rcall	Start_IO							;if external Sensor is used, get temperature from external sensor and exit temperature calculation 
 rjmp	_CT22

_CalcT11:  
 ldi	Temp3, low(12000)					;load multiplier 1000 x R1(in kOhms)
 mov	Ch0, Temp3
 ldi	Temp3, high(12000)
 mov	Ch1, Temp3
 
 lds	Temp3, iNTCVL						;Load U1
 lds	Temp4, iNTCVH
 sub	Temp3, Temp
 sbc	Temp4, TempH


 mul	TempH, Ch1							;ah*bh		multiply with r1
 movw	Di3:Di2, r1:r0
 mul	Temp, Ch0							;al*bl
 movw	Di1:Di0, r1:r0
 mul	TempH, Ch0							;ah*bl
 add	Di1, r0
 adc	Di2, r1
 adc	Di3, Null
 mul	Ch1, Temp							;bh*al
 add	Di1, r0
 adc	Di2, r1
 adc	Di3, Null

 ldi	Temp5, 1
 clr	Temp 
 clr	TempH

_TempDivLoop:								;U2/U1
 sub	Di0, Temp3
 sbc	Di1, Temp4
 sbc	Di2, Null
 sbc	Di3, Null
 brcs	_TempDivEnd
 add	Temp, Temp5
 adc	TempH, Null
 rjmp	_TempDivLoop
 
_TempDivEnd:						
 ldi	ZH, high(NTCIntTable*2)
 ldi	ZL, low(NTCIntTable*2)
 lpm	Temp3, Z+
 lpm	Temp4, Z+
 sbiw	ZH:ZL, 2
 cp 	Temp3, Temp 						;if table value is lower then read value, set temp to 0.00°C and skip
 cpc	Temp4, TempH
 brcc	_CT0
 clr	Di0				
 clr	Di1
 rjmp	_CT21

_CT0:
 ldi	Temp5, 50
 ldi	Temp3, (-50)						;init with -5°C
 mov	Di0, Temp3
 ldi	Temp3, (-1)
 mov	Di1, Temp3


_CT1:
 lpm	Temp3, Z+
 lpm	Temp4, Z+
 
 cpi	Temp3, 0							;$0000 table end mark
 cpc	Temp4, Null
 breq	_TempTop
   
 cp 	Temp3, Temp 						;if table value is lower then read value, skip to fine reolution
 cpc	Temp4, TempH
 brcs	_CT2
 
 mov	Di2, Temp3							;store coarse value for later use
 mov	Di3, Temp4

 add	Di0, Temp5							;temperature coarse steps of 5.0°
 adc	Di1, Null
 rjmp	_CT1


_TempTop:
 ser	Temp
 mov	Di0, Temp
 mov	Di1, Temp
 rjmp	_CT21


_CT2: 
 push	Di2
 push	Di3

 sub	Di2, Temp3							;values between two 5°steps
 sbc	Di3, Temp4
 ldi	Temp3, 1
 mov	Ch0, Temp3
 clr	Temp3
 clr	Temp4

_TempDivLoop2:
 sub	Di2, Temp5							;divide values with 50, 0.1° resolution
 sbc	Di3, Null
 brcs	_TempDivL2End
 add	Temp3, Ch0
 adc	Temp4, Null
 rjmp	_TempDivLoop2
 
_TempDivL2End:
 ldi	Temp5, 1
 pop	Di3
 pop	Di2

_TempFineLoop:
 sub	Di2, Temp3
 sbc	Di3, Temp4
 cp		Di2, Temp
 cpc	Di3, TempH
 brlt	_CT21
 add	Di0, Temp5
 adc	Di1, Null
 rjmp	_TempFineLoop
   
_CT21:
 sts   	TempIntL, Di0
 sts	TempIntH, Di1

 
/* in		Temp, Status2
 sbr	Temp, (1<<_TX_SSPI)
 out	Status2, Temp*/

_CT22:
 sbis	Status0, _Adapt
 rcall	Regulate
 call	Measure_Battery_Voltage 
 ret






;--------------------------------------------------------------------------------------------------
;change one byte of binary format to 2 bytes ASCII HEX format, used for LCD

DivHex:
   
DivH:					
 mov 	TempH, Temp
 andi	TempH, $F0
 swap	TempH
 mov	Di1, TempH
 ldi	TempH, $30
 add	Di1, TempH
 ldi	TempH, $3A
 cp		Di1, TempH
 brcs	DivL
 ldi	TempH, 7
 add	Di1, TempH

DivL:					
 mov 	TempH, Temp
 andi	TempH, $0F
 mov	Di0, TempH
 ldi	TempH, $30
 add	Di0, TempH
 ldi	TempH, $3A
 cp		Di0, TempH
 brcs	DivHEnd
 ldi	TempH, 7
 add	Di0, TempH

DivHEnd:					
 ret


;--------------------------------------------------------------------------------------------------
;change two byte of binary (High/Low) format to 4 bytes ASCII DEC format, used for LCD

DivHL:
 clr	Di0				; Alle temporären Dezimalstellen löschen
 clr	Di1
 clr	Di2
 clr	Di3 

DivHL2:					    
 cpi	Temp, low(1000)		;Compare low byte
 ldi	Temp3, high(1000)	;
 cpc	TempH, Temp3	    ;Compare high byte
 brlo	DivHL3
 subi	Temp, low(1000)		;Subtract low bytes
 sbci	TempH, high(1000)	;Subtract high byte with carry
 inc	Di3
 rjmp	DivHL2

DivHL3:					    
 cpi	Temp, low(100)		;Compare low byte
 ldi	Temp3, high(100)	;
 cpc	TempH, Temp3	    ;Compare high byte
 brlo	DivHL4
 subi	Temp, low(100)		;Subtract low bytes
 sbci	TempH, high(100)	;Subtract high byte with carry
 inc	Di2
 rjmp	DivHL3

DivHL4:				
 cpi 	Temp, 10
 brlo 	DivHLEnd
 subi	Temp, 10
 inc	Di1
 rjmp	DivHL4

DivHLEnd:				
 mov	Di0, Temp

 ldi	Temp, 48		;ASCII-Zeichenumwandlung
 add	Di3, Temp
 add	Di2, Temp
 add	Di1, Temp
 add	Di0, Temp
 ret



;--------------------------------------------------------------------------------------------------
;Teilt 8_Bit Wert in 3-stellige Dezimalzahl

DivDez:
 clr	Di0				; Alle temporären Dezimalstellen löschen
 clr	Di1
 clr	Di2
 
Div1:					    
 cpi	Temp, 100	
 brlo	Div2
 subi	Temp, 100
 inc	Di2
 rjmp	Div1

Div2:				
 cpi 	Temp, 10
 brlo 	DivE
 subi	Temp, 10
 inc	Di1
 rjmp	Div2

DivE:				
 mov	Di0, Temp

DivE1:
 ldi	Temp, 48
 add	Di2, Temp
 add	Di1, Temp
 add	Di0, Temp
 ret

;-------------------------------------------------
;divides Timer value 0...143 to 3 digits for Hours and tenth minutes 

DivTimer2HourTenMin:
 clr	Di2
 clr	Di1
 clr	Di0				

DT2HTM_1:
 cpi	Temp, 60
 brlo	DT2HTM_2
 subi	Temp, 60
 inc	Di2
 rjmp	DT2HTM_1 

DT2HTM_2:
 cpi	Temp, 6
 brlo	DT2HTM_3
 subi	Temp, 6
 inc	Di1
 rjmp	DT2HTM_2 

DT2HTM_3:
 mov	Di0, Temp
 rjmp	DivE1

;--------------------------------------------------------
;changes Hours and tens of Minutes into TimerValue 0..144
;Temp contains Minutes, TempH contains Hours
HourTenMin2TimerValue:
 ldi	Temp3, 6
 mul	TempH, Temp3
 mov	Di0, r0

_HTM2TV1:
 cpi	Temp, 10
 brlo	_HTM2TV2
 subi	Temp, 10
 inc	Di0
 rjmp	_HTM2TV1


_HTM2TV2:
 ret


;----------------------------------------------------------------------
TimerHour2Bin:
 clr	Di0

_TH2B1:
 cpi	Temp, 6
 brlo	_TH2B2
 subi	Temp, 6
 inc	Di0
 rjmp	_TH2B1

_TH2B2:
 mov	Temp, Di0
 ret

 


;----------------------------------------------------------------------
;Lesen eines Wertes aus dem internen EEPROM
;Adresse des zu lesenden Wertes steht in Temp3 / TempH
;gelesener Wert wird in Temp zurückgegeben

EERead:
 sbic	EECR, EEWE		;if EEWE not clear
 rjmp	EERead			;    wait more
 cli
 out	EEARH, Temp3	;output address
 out	EEARL, TempH
 sbi	EECR, EERE		;set EEPROM Read strobe
 in		Temp, EEDR		;get data
 sei
 ret

;--------------------------------------------------------------------------------------------------
;Schreiben eines Wertes in das interne EEPROM
;Adresse des zu schreibenden Wertes steht in Temp3 / TempH
;zu schreibender Wert steht in Temp 

EEWrite:
 sbic	EECR, EEWE		;if EEWE not clear
 rjmp	EEWrite			;wait more
 cli
 out	EEARH, Temp3	;output address
 out	EEARL, TempH
 out	EEDR, Temp		;put data
 sbi	EECR, EEMWE		;set MasterWriteEnable
 sbi	EECR, EEWE		;set EEPROM Write strobe
 sei
 ret



.include	"Regulation.inc"
.include	"LCD_Driver.inc"		;Display handler
.include	"LCDTable.inc"
.include	"Debug.inc"
.include	"Motor.inc"
.include	"User.inc"
.include	"ISR.inc"
.include	"ADC.inc"
.include	"RTC.inc"






NTCIntTable:
.dw	34090		;  0°C
.dw	26310		;  5°C
.dw 20440		; 10°C
.dw	16000		; 15°C
.dw	12610		; 20°C
.dw	10000		; 25°C
.dw	 7981		; 30°C
.dw	 6408		; 35°C
.dw	 5174		; 40°C
.dw	 4202		; 45°C
.dw	 3431		; 50°C
.dw	 2816		; 55°C
.dw  2322		; 60°C
.dw	 1925		; 65°C
.dw	 1603		; 70°C
.dw	 1340		; 75°C
.dw	 1126		; 80°C
.dw	  949		; 85°C
.dw	  804		; 90°C
.dw	  684		; 95°C
.dw	  	0






MenuTable:
.db $00, $00				;MODE   
	.db	$00, $10			;MANU   
		.db	$00, $11		;MEnt	
	.db $00, $20			;AUTO  
		.db $00, $21		;AEnt	
.db $01, $00				;PROG	
	.db	$01, $10 			;TAG1	
		.db	$01, $11		
		.db	$01, $12		
		.db	$01, $13		
		.db	$01, $14		
		.db	$01, $15
		.db	$01, $16		
		.db	$01, $17		
		.db	$01, $18		
		.db	$01, $19		
	.db	$01, $20			;TAG2	 
		.db	$01, $21		
		.db	$01, $22		
		.db	$01, $23		
		.db	$01, $24		
		.db	$01, $25
		.db	$01, $26		
		.db	$01, $27		
		.db	$01, $28		
		.db	$01, $29		 
	.db	$01, $30			;TAG3	
		.db	$01, $31		
		.db	$01, $32		
		.db	$01, $33		
		.db	$01, $34		
		.db	$01, $35
		.db	$01, $36		
		.db	$01, $37		
		.db	$01, $38		
		.db	$01, $39		 
	.db	$01, $40			;TAG4 
		.db	$01, $41		
		.db	$01, $42		
		.db	$01, $43		
		.db	$01, $44		
		.db	$01, $45
		.db	$01, $46		
		.db	$01, $47		
		.db	$01, $48		
		.db	$01, $49
	.db	$01, $50			;TAG5 
		.db	$01, $51		
		.db	$01, $52		
		.db	$01, $53		
		.db	$01, $54		
		.db	$01, $55
		.db	$01, $56		
		.db	$01, $57		
		.db	$01, $58		
		.db	$01, $59
	.db	$01, $60			;TAG6
		.db	$01, $61		
		.db	$01, $62		
		.db	$01, $63		
		.db	$01, $64		
		.db	$01, $65
		.db	$01, $66		
		.db	$01, $67		
		.db	$01, $68		
		.db	$01, $69		
	.db	$01, $70			;TAG7	
		.db	$01, $71		
		.db	$01, $72		
		.db	$01, $73		
		.db	$01, $74		
		.db	$01, $75
		.db	$01, $76		
		.db	$01, $77		
		.db	$01, $78		
		.db	$01, $79		
	.db $01, $80			;T1-5	
		.db	$01, $81		
		.db	$01, $82		
		.db	$01, $83		
		.db	$01, $84		
		.db	$01, $85
		.db	$01, $86		
		.db	$01, $87		
		.db	$01, $88		
		.db	$01, $89		
	.db	$01, $90			;T1-6
		.db	$01, $91		
		.db	$01, $92		
		.db	$01, $93		
		.db	$01, $94		
		.db	$01, $95
		.db	$01, $96		
		.db	$01, $97		
		.db	$01, $98		
		.db	$01, $99	
	.db	$01, $A0			;T1-7
		.db	$01, $A1		
		.db	$01, $A2		
		.db	$01, $A3		
		.db	$01, $A4		
		.db	$01, $A5
		.db	$01, $A6		
		.db	$01, $A7		
		.db	$01, $A8		
		.db	$01, $A9	
	.db	$01, $B0			;Urlaub/Ferien
		.db	$01, $B1		
		.db	$01, $B2		
		.db	$01, $B3		
		.db	$01, $B4		
		.db	$01, $B5
		.db	$01, $B6		
		.db	$01, $B7		
		.db	$01, $B8		
		.db	$01, $B9		
.db $02, $00				;TEMP	
	.db	$02, $10			;onTemp 	
	.db	$02, $20			;offTemp	
	.db	$02, $30			;NightTemp
.db $03, $00				;ZEIT	 
	.db	$03, $10			;set year		 
	.db	$03, $20			;set month		 
	.db	$03, $30			;set date		 
	.db	$03, $40			;set hour		 
	.db	$03, $50			;set minute		 
.db $04, $00				;FENS			 
	.db	$04, $10			;HOCH			 
		.db $04, $11		;time resume	 
	.db	$04, $20			;MITT			 
		.db $04, $21		;time resume	 
	.db	$04, $30			;NIED			 
		.db $04, $31		;time resume	 
.db  $05, $00				;RESET			 
	.db	 $05, $10			;OK				 
.db $06, $00				;ADAP			 
	.db	$06, $10			;ADAP			 
.db $07, $00				;URLA			 
	.db	$07, $10			;set holiday 1	 
		.db 0x07, 0x11
	.db	$07, $20			;set holiday 2		 
		.db 0x07, 0x21
	.db	$07, $30			;set holiday off	 
		.db 0x07, 0x31
.db $08, $00				;INST			 
	.db	$08, $10			;NOR			 
		.db	$08, $11		;NorEnt
	.db	$08, $20			;INV			 
		.db	$08, $21		;InvEnt
.db $09, $00				;OFFS			 
	.db	$09, $10			;set temp		 
.db $0A, $00				;DBUG			 
	.db	$0A, $10			;FW				 
	.db	$0A, $20			;Fuzz			 
	.db	$0A, $30			;Olim			 
	.db	$0A, $40			;VTop
	.db	$0A, $50			;RWay
	.db	0x0A, 0x60			;VBAT

.db $FF, $FF				;menu end



//End of file//