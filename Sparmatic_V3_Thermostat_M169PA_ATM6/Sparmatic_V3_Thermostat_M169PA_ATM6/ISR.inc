;--------------------------------------------------
;Sound interrupt
T0_CMP:
 in		SREG_Save, SREG
 push 	Temp
 push 	TempH
 
 lds	Temp, BeepPulseCT
 inc	Temp
 sts	BeepPulseCT, Temp
 cpi	Temp, 3
 brcs	_BeepPulse1
 breq	_BeepPulseOff
 cpi	Temp, 6
 brcs	_BeepPulse2

_BeepPulseOff:
 cbi	MotorDDR, Motor_Open
 cbi	MotorPort, Motor_Open
 cbi	MotorDDR, Motor_Close
 cbi	MotorPort, Motor_Close
 rjmp	_BeepInt1
 
_BeepPulse1:
 cbi	MotorDDR, Motor_Close
 cbi	MotorPort, Motor_Close
 sbi	MotorDDR, Motor_Open
 sbi	MotorPort, Motor_Open
 rjmp	_BeepInt1

_BeepPulse2:
 cbi	MotorDDR, Motor_Open
 cbi	MotorPort, Motor_Open
 sbi	MotorDDR, Motor_Close
 sbi	MotorPort, Motor_Close

_BeepInt1:
 cpi	Temp, 15
 brcs	_BeepEnd
  
 sts	BeepPulseCT, Null
 lds	Temp, BeepCT
 lds	TempH, BeepLen
 cp		Temp, TempH
 brcc	_BeepOff
 inc	Temp
 sts	BeepCT, Temp
 rjmp	_BeepEnd

_BeepOff:
 lds	Temp, TIMSK0
 cbr	Temp, (1<<OCIE0A)
 sts	TIMSK0, Temp
 
_BeepEnd:
 pop	TempH
 pop	Temp
 out	SREG, SREG_Save
 reti




;Reflex coupler pin change interrupt
;--------------------------------------------------
PinChange0:
 sbis	Status0, _MotRun						;execute only if motor hardware is on
 reti 
 
 ldi	ActiveCT, 3

 in		SREG_Save, SREG
 push 	XL
 push 	XH

 sbic	LightSensPin, LightSens_Trans			;count only falling edge
 rjmp	_PC0_2

 sbis	Status0, _Adapt
 rjmp	_PC0_1
 
 lds	XL, PositionL							;decrement Reflex counter down to 0
 lds	XH, PositionH
 adiw	XH:XL, 1
 sts	PositionL, XL
 sts	PositionH, XH

_PC0_1:
 lds	XL, RFLXCTL								;decrement Reflex counter down to 0
 lds	XH, RFLXCTH
 cpi	XL, 0
 cpc	XH, Null
 breq	_PC0_2
 sbiw	XH:XL, 1
 sts	RFLXCTL, XL
 sts	RFLXCTH, XH

_PC0_2:
 lds	XL, MotTimeOut
 andi	XL, 0b11000000							;clear motor timeout counter
 sts	MotTimeOut, XL
 
 pop	XH
 pop	XL
 out	SREG, SREG_Save
 reti


;save current RTC to EEPROM
;--------------------------------------------------
RapidShutOff:
 cbi	PORTE, 5										;disable pullup on test pin
 cbi	PortB, 7										;disable Pullups on rotary encoder
 cbi	PortB, 0
 
 sts	ADCSRA, Null									;disable ADC

 cbi	MotorDDR, Motor_Close
 cbi	MotorPort, Motor_Close
 cbi	MotorDDR, Motor_Open
 cbi	MotorPort, Motor_Open
 cbi	LightSensPort, LightSens_LED					;disable reflex coupler
 
 call	Store_Time
 call	Clear_Screen

 ldi	Temp, 'O'
 ldi	TempH, $30	
 call	PutCharacter
 ldi	Temp, 'F'
 call	PutCharacter
 ldi	Temp, 'F'
 call	PutCharacter
 ldi	Temp, '?'					 
 call	PutCharacter
 call	Copy_DisplayBuffers 
 
 ldi	Temp, (1<<SM1) | (1<<SM0) | (1<<SE)	;Power save sleep mode
 out	SMCR, Temp

 sleep

OffLoop:
 rjmp	OffLoop



;Buttons pin change interrupt
;--------------------------------------------------
PinChange1:
 in		SREG_SAVE, SREG
 push	Temp
 push	TempH

 sbic	IO_PIN, IO_SCK									;if IO clock is low, initiate data read
 rjmp	_PC1_1
  
 clc
 sbic	IO_PIN, IO_SDATA					;if data pin is high, rotate a logical "1"
 sec										
 lds	Temp, IO_Data
 ror	Temp								;assemble bits
 sts	IO_Data, Temp
 lds	TempH, IO_BitCT
 inc	TempH
 sts	IO_BitCT, TempH
 andi	TempH, 0b00000111
 brne	_PC1_1
 
 sts	IO_DataBuffer, Temp
 in		Temp, Status1
 sbr	Temp, (1<<_IO_Receive)
 out	Status1, Temp
 
_PC1_1:
 ldi	Temp, (1<<LCDEN) | (1<<LCDIE) | (1<<LCDAB)		;enable LCD-Frame Interrupt
 sts	LCDCRA, Temp
 ldi	ActiveCT, 3
 call	ReadRotary
 
 pop	TempH
 pop	Temp
 out	SREG, SREG_SAVE
 reti


;LCD-controller frame Interrupt
;--------------------------------------------------
LCD_FrameInt:
 in		SREG_SAVE, SREG
 push	Temp
 push	TempH

 in		Temp, PinB
 com	Temp
 andi	Temp, 0b11110001			;mask used pins and swap nibbles, put all button pins together -> 000bbbbb
 swap	Temp
 andi	Temp, 0b00000111			;mask buttons
 cpi	Temp, 0						;if no button is pressed, clear button value and debounce counter
 breq	_NoButton
 
 lds	TempH, OldButtons
 sts	OldButtons,	Temp
 cpse	Temp, TempH
 rjmp	_LCDFInt1

_ButtonEqual:
 cpi	ButtonDebounce, 0xFF
 breq	_LCDFInt1
 inc	ButtonDebounce
 cpi	ButtonDebounce, 5
 brne	_LCDFInt1

 sts	Buttons, Temp 
 sbi	Status0, _NewButton
 rjmp	_LCDFInt1

_NoButton:
 clr	ButtonDebounce	
 sts	OldButtons, Null
 
_LCDFInt1: 
 lds	Temp, MotTimeout								;increment motor timeout counter each LCD frame
 andi	Temp, $3F
 cpi	Temp, $3F
 breq	_LCDF_E
 lds	Temp, MotTimeout
 inc	Temp
 sts	MotTimeOut, Temp
  
_LCDF_E:
 pop	TempH
 pop	Temp
 out	SREG, SREG_SAVE
 reti
 


;--------------------------------------------------
SystemTime:
 in		SREG_SAVE, SREG
 push 	Temp
 push	TempH
 push 	Temp3

 lds	Temp, DispTimer
 inc	Temp
 andi	Temp, 0b00000011
 sts	DispTimer, Temp
 brne	_SysT1

 lds	Temp, DisplayCT							;used for alternating display content, seconds timing
 inc	Temp
 sts	DisplayCT, Temp
 lds	Temp, UserDisplay
 tst	Temp
 breq	_SysT1 
 dec	Temp
 sts	UserDisplay, Temp
 brne	_SysT1 
 sts	DisplayCT, Null
 
_SysT1:
 sbic	Status0, _Adapt
 rjmp	_Sys_T11

 lds	Temp, UserSettings1						;if external sensor is used, start I/O-Routine
 sbrs	Temp, _ExternalTempSensor
 rjmp	_Sys_T11

 lds	Temp, Radio_RXTimeslot
 cpi	Temp, 0
 breq	_Sys_T11
 
 dec	Temp
 sts	Radio_RXTimeslot, Temp
 brne	_Sys_T11

 call	Start_IO								;wake up external RFM tray

_Sys_T11:
 lds	Temp, PSC1
 inc	Prescaler1
 cp		Prescaler1, Temp
 breq	_SysTSeconds
 rjmp	_SysTE
  
_SysTSeconds:									;1s timer
 ;sbi	DebugPort, Debug1
 
 clr	Prescaler1
 ldi	Temp, 4									;restore prescaler interval after time adjustment
 sts	PSC1, Temp
 in 	Temp, Status1
 sbr	Temp, (1<<_SecondTick)
 out	Status1, Temp

 sbic	Status0, _Adapt
 rjmp	_SysT2
 
 lds	Temp, ADCPrescaler
 dec	Temp
 sts	ADCPrescaler, Temp
 brne	_SysT2

 ldi	Temp, 180									;3 Minutes measure interval
 sts	ADCPrescaler, Temp

 in 	Temp, Status1
 sbr 	Temp, (1<<_NewTemp)
 out	Status1, Temp

  
_SysT2: 
 cpi	ActiveCT, 0
 breq	_SysTE
 dec	ActiveCT
 brne	_SysTE

 ldi	Temp, (1<<LCDEN) | (0<<LCDIE)| (1<<LCDAB)	;Disable LCD-Frame-ISR
 sts	LCDCRA, Temp
 cbi	BacklightPort, Backlight_On
 
 ;cbi	DebugPort, Debug2


_SysTE:
 ;cbi	DebugPort, Debug1
 ;cbi	DebugPort, Debug2
 pop	Temp3
 pop	TempH
 pop	Temp
 out	SREG, SREG_SAVE
 reti


 //End of file//