;--------------------------------------------------
;Sound interrupt
T0_CMP:
 in		SREG_Save, SREG
 push 	Temp
 push 	TempH
 
 lds	Temp, BeepPulseCT
 inc	Temp
 sts	BeepPulseCT, Temp
 cpi	Temp, 10
 brcs	_BeepPulse1
 cpi	Temp, 10
 breq	_BeepPulseOff
 cpi	Temp, 20
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
 cpi	Temp, 40
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
 sbic	PinE, 0
 rjmp	RapidShutOff
 
 sbis	Status0, _MotRun						;execute only if motor hardware is on
 reti 
 
 in		SREG_Save, SREG
 push 	XL
 push 	XH

 lds	XL, RFLXCTL
 lds	XH, RFLXCTH

 sbis	Status0, _MotDir
 rjmp	_RFLX_Open
 
_RFLX_Close:
 sbic	LightSensPin, LightSens_Trans			;while closing valve, count pulses only on H/L edge (dark -> light)
 rjmp	_RFLX_On
 adiw	XH:XL, 1
 rjmp	_RFLX_On

_RFLX_Open:
 sbis	LightSensPin, LightSens_Trans			;while opening valve, count pulses only on L/H edge (light -> dark)
 rjmp	_RFLX_On
 sbiw	XH:XL, 1
  
_RFLX_On:
 sts	RFLXCTL, XL
 sts	RFLXCTH, XH

 sbis	Status0, _MotOn							;execute only if motor is running
 rjmp	_PC0_1
 
 lds	XH, PositionL							;if current position is equal to calculated position, set equal flag
 cp		XL, XH
 brne	_PC0_1
 lds	XL, RFLXCTH 
 lds	XH, PositionH
 cp		XL, XH
 brne	_PC0_1

 lds	XL, MotTimeOut
 sbr	XL, (1<<_PosEqual)
 sts	MotTimeOut, XL

 sbi	DebugPort, Debug2

 
_PC0_1:
 lds	XL, MotTimeOut
 andi	XL, 0b11000000
 sts	MotTimeOut, XL
 
 pop	XH
 pop	XL
 out	SREG, SREG_Save
 reti


;save current RTC to EEPROM
;--------------------------------------------------
RapidShutOff:
 cbi	DebugPort, Debug1
 cbi	PortB, 7							
 cbi	PortB, 0
 
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

 ldi	Temp, (1<<LCDEN) | (1<<LCDIE) | (1<<LCDAB)		;enable LCD-Frame Interrupt
 sts	LCDCRA, Temp
 ldi	ActiveCT, 3
 sbi	BacklightPort, Backlight_On

 rcall	ReadButtons
  
 pop	TempH
 pop	Temp
 out	SREG, SREG_SAVE
 reti


;--------------------------------------------------
LCD_FrameInt:
 in		SREG_SAVE, SREG
 push	Temp
 
 lds	Temp, MotTimeout
 andi	Temp, $3F
 cpi	Temp, $3F
 breq	_LCDF_E
 lds	Temp, MotTimeout
 inc	Temp
 sts	MotTimeOut, Temp
  
_LCDF_E:
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

 lds	Temp, DisplayCT					;used for alternating display content, seconds timing
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
 lds	Temp, PSC1
 inc	Prescaler1
 cp		Prescaler1, Temp
 breq	_SysTSeconds
 rjmp	_SysTE
  
_SysTSeconds:							;1s timer
 ;sbi	DebugPort, Debug1
 
 clr	Prescaler1
 ldi	Temp, 4							;restore prescaler interval after time correction
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

 ldi	Temp, 120						;2 Minutes measure interval
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
 
 cbi	DebugPort, Debug2


_SysTE:
 ;cbi	DebugPort, Debug1
 ;cbi	DebugPort, Debug2
 pop	Temp3
 pop	TempH
 pop	Temp
 out	SREG, SREG_SAVE
 reti
