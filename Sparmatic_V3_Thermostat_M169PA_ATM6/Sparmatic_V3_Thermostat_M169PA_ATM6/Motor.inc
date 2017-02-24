
;--------------------------------------------------
Adaptation:
 sbis	Status0, _Adapt						;execute in adaptation mode only
 ret

 sbis	Status0, _MotDir
 rjmp	_Adapt3

_Adapt0:
 lds	Temp, RFLXCTL
 lds	TempH, RFLXCTH

 cpi	Temp, low(MaxAdaptWay-20)			;get freewheeling motor current on outer position -n
 ldi	Temp3, high(MaxAdaptWay-20)
 cpc	TempH, Temp3
 brne	_Adapt1

 call	Measure_Motor_Current

 lds	Temp3, MotorCurrent
 sts	FreeMotorCurrent, Temp3
 lds	Temp3, DisplayBuffer1+11			;set bargraph point 1, freewheeling current measured
 ori	Temp3, 0b00000001
 sts	DisplayBuffer1+11, Temp3
 lds	Temp3, DisplayBuffer2+11						 
 ori	Temp3, 0b00000001
 sts	DisplayBuffer2+11, Temp3
 
_Adapt1:
 lds	Temp3, MotorCurrent
 lds	Temp4, FreeMotorCurrent
 sub	Temp3, Temp4
 brcc	_Adapt2
 clr	Temp3

_Adapt2:
 cpi	Temp3, 10							;threshold current over freewheeling current, to detect valve touch
 brcs	_Adapt3

 lds	Temp, DisplayBuffer1+6				;execute only once if the current rises above threshold
 sbrc	Temp, 0
 rjmp	_Adapt2_1

 lds	Temp3, MotorCurrent
 sts	ValveMotorCurrent, Temp3
 
 sts	PositionL, Null
 sts	PositionH, Null

 lds	Temp, DisplayBuffer1+6				;set bargraph point 2, touch current measured
 ori	Temp, 0b00000001
 sts	DisplayBuffer1+6, Temp
 lds	Temp, DisplayBuffer2+6					
 ori	Temp, 0b00000001
 sts	DisplayBuffer2+6, Temp
 
_Adapt2_1:
 ldi	Temp, $10							;keep Reflex counter well above 0
 ldi	TempH, $00
 sts	RFLXCTL, Temp
 sts	RFLXCTH, TempH
 

_Adapt3:
 sbic	Status0, _MotRun
 ret 
 
 lds	TempH, AdaptStep
 cpi	TempH, 0
 breq	_AdpOpenValve
 cpi	TempH, 2
 breq	_WaitADP_OK
 cpi	TempH, 4
 breq	_AdpCloseValve
 cpi	TempH, 6
 breq	_AdpEndX 
 ret

_AdpEndX:
 rjmp	_AdpEnd

  
_AdpOpenValve:
 cbi	Status0, _MotDir
 sbi 	Status0, _MotOn	
 sts	MotTimeOut, Null
 inc	TempH
 sts	AdaptStep, TempH
 ldi	Temp, '<'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '<'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 ldi	TempH, $20
 rcall	PutCharacter
 ldi	Temp, '<'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '<'
 rcall	PutCharacter
 ldi	Temp, $10
 sts	RFLXCTH, Temp
 ldi	Temp, 0xFF
 sts	ValveMotorCurrent, Temp
 ret

_WaitADP_OK:
 in		Temp, Status1
 sbrc	Temp, _Descale
 rjmp	_Descale1

 sbis	Status0, _NewButton

_WADPO_E:
 ret
 
 cbi	Status0, _NewButton
 lds	Temp, Buttons
 cpi	Temp, _OK_Button
 brne	_WADPO_E
 ldi	Temp, 4
 sts	AdaptStep, Temp
 ldi	Temp, 'A'
 ldi	TempH, $30
 rcall	PutCharacter
 ldi	Temp, 'D'
 rcall	PutCharacter
 ldi	Temp, 'A'
 rcall	PutCharacter
 ldi	Temp, 'P'
 rcall	PutCharacter
 sts	PositionL, Null
 sts	PositionH, Null
 ret


_AdpCloseValve:
 sbi	Status0, _MotDir
 sbi 	Status0, _MotOn	
 sts	MotTimeOut, Null
 inc	TempH
 sts	AdaptStep, TempH
 ldi	Temp, low(MaxAdaptWay)					;preload maximum way without valve touch
 ldi	TempH, high(MaxAdaptWay)
 sts	RFLXCTL, Temp
 sts	RFLXCTH, TempH
 ret
 

_Descale1:
 ldi	Temp, 4
 sts	AdaptStep, Temp
 ldi	Temp, 'D'
 ldi	TempH, $30
 rcall	PutCharacter
 ldi	Temp, 'E'
 rcall	PutCharacter
 ldi	Temp, 'S'
 rcall	PutCharacter
 ldi	Temp, 'C'
 rcall	PutCharacter
 sts	PositionL, Null
 sts	PositionH, Null
 cli
 in		Temp, Status1
 cbr	Temp, (1<<_Descale)
 out	Status1, Temp
 sei
 ret 


_AdpEnd:
 sts	AdaptStep, Null

 lds	Temp, DisplayBuffer1+11					;clear bargraph points 1 and 2
 andi	Temp, 0b11111110
 sts	DisplayBuffer1+11, Temp
 lds	Temp, DisplayBuffer2+11					
 andi	Temp, 0b11111110
 sts	DisplayBuffer2+11, Temp
  
 lds	Temp, DisplayBuffer1+6
 andi	Temp, 0b11111110
 sts	DisplayBuffer1+6, Temp
 lds	Temp, DisplayBuffer2+6
 andi	Temp, 0b11111110
 sts	DisplayBuffer2+6, Temp
 
 lds	Temp, PositionL
 lds	TempH, PositionH
 subi	Temp, 20								;subtact margin to full open position
 sbc	TempH, Null
 sts	ValveTopL, Temp
 sts	ValveTopH, TempH

 sts	PositionL, Null
 sts	PositionH, Null
 
 ldi	Temp, 40
 sts	ADCPrescaler, Temp 
 sts	UserDisplay, Null
 sts	DisplayCT, Null
 sts	DispTimer, Null
 
 call	CalcDayOfWeek
 ori	Temp, $80
 rcall	ClearWeekDays
 ldi	TempH, $30
 rcall	PutWeekDay
  
 in		Temp, Status1
 sbrs	Temp, _AutoManu
 rcall	Show_DailyBargraph
 
 in		Temp, Status1
 sbrs	Temp, _AutoManu
 rcall	SetHourLegend
   
 rcall	Show_AutoManuMode

 cbi	Status0, _Adapt
 
 ldi	Temp, 3
 sts	Radio_RXTimeslot, Temp

 in		Temp, Status2
 sbr	Temp, (1<<_FastModeScan)				;enable fast timer scan
 out	Status2, Temp

 ret
 
 



;------------------------------------------------------------------------------
MotorControl:
 lds	Temp, MotTimeOut					
 andi	Temp, 0b00111111
 cpi	Temp, 30										;timeout value to completely shut down motor hardware, if lower, execute motor control
 brcs	_MC2
 cbi	Status0, _MotRun					
 cbi	LightSensPort, LightSens_LED					;switch off reflex sensor
 
 lds	Temp, MotTimeOut								;if any valve limit is touched, clear reflex counter 
 andi	Temp, 0b11000000
 breq	_MC1

 sts	RFLXCTL, Null									
 sts	RFLXCTH, Null
 
_MC1: 
 ret
  
_MC2: 
 sbis	Status0, _MotOn									;exit, if motor has been already stopped
 ret
 
 sbis	Status0, _MotDir								;measure motor current only when closing valve
 rjmp	_MC22

 call	Measure_Motor_Current

 lds	Temp, MotTimeOut
 andi	Temp, 0b00111111
 lds	Temp3, MotorCurrent								;compare actual current to detect overcurrent when valve is fully closed
 lds	Temp4, ValveMotorCurrent
 sub	Temp3, Temp4
 brcc	_MC21
 clr	Temp3

_MC21:
 cpi	Temp3, 40										;threshold current over valvecurrent, overcurrent!
 brcc	_MC23

_MC22:
 lds	Temp, MotTimeOut 
 andi	Temp, 0b00111111
 cpi	Temp, 10										;timeout value to shut down motor, if lower, execute motor control
 brcs	_MC3	

_MC23:  
 sbic	Status0, _MotDir
 sbr	Temp, (1<<_BotLimit)
 sbis	Status0, _MotDir
 sbr	Temp, (1<<_TopLimit)
 sts	MotTimeOut, Temp 
 sbrs	Temp, _BotLimit									; if valve is fully closed, clear position counter
 rjmp	_MotorStop
 sbic	Status0, _Adapt
 rjmp	_MotorStop
 sts	PositionL, Null
 sts	PositionH, Null
 rjmp	_MotorStop

_MC3:   
 lds	Temp, RFLXCTL									;check if position is reached
 lds	TempH, RFLXCTH
 cpi	Temp, 0
 cpc	TempH, Null
 breq	_MotorStop

 sbic	Status0, _MotRun								;execute only if motor hardware was stopped before
 ret
 
 sbi	LightSensPort, LightSens_LED					;enable reflex coupler

 ldi	Temp, (1<<LCDEN) | (1<<LCDIE) | (1<<LCDAB)		;enable LCD-Frame Interrupt, used for upcounting Motor timeout
 sts	LCDCRA, Temp
 ldi	ActiveCT, 3
 sbi	BacklightPort, Backlight_On

 sbis	Status0, _MotDir								;dependent on direction flag open or close valve
 rjmp	_MotorOpen										;flag is clear: open, flag is set: close

_MotorClose:
 lds	Temp, MotTimeOut								;clear opposite limit flag
 cbr	Temp, (1<<_TopLimit)
 sts	MotTimeOut, Temp
 ;cbi	DebugPort, Debug2
 cbi	MotorDDR, Motor_Open
 cbi	MotorPort, Motor_Open
 nop
 nop
 sbi	MotorDDR, Motor_Close
 sbi	MotorPort, Motor_Close
 sbi	Status0, _MotRun
 ret

_MotorOpen:
 lds	Temp, MotTimeOut								;clear opposite limit flag
 cbr	Temp, (1<<_BotLimit)
 sts	MotTimeOut, Temp
 ;cbi	DebugPort, Debug2
 cbi	MotorDDR, Motor_Close
 cbi	MotorPort, Motor_Close
 nop
 nop
 sbi	MotorDDR, Motor_Open
 sbi	MotorPort, Motor_Open
 sbi	Status0, _MotRun
 ret
  
_MotorStop:
 sbis	Status0, _MotOn									;if motor has been stopped before, exit
 ret

 cbi	MotorDDR, Motor_Close
 cbi	MotorPort, Motor_Close							;disable both motor drivers
 cbi	MotorDDR, Motor_Open
 cbi	MotorPort, Motor_Open
 cbi	Status0, _MotOn						

 sbis	Status0, _Adapt									;in adaptation mode, increase adaptation step
 ret

 lds	Temp, AdaptStep
 cpi	Temp, 1
 brne	_MStop_1
 

 ldi	Temp, 'I'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, 'N'
 rcall	PutCharacter
 ldi	Temp, 'S'
 rcall	PutCharacter
 ldi	Temp, 'T'
 rcall	PutCharacter
 
 ldi	Temp, '?'
 ldi	TempH, $20
 rcall	PutCharacter
 ldi	Temp, 'O'
 rcall	PutCharacter
 ldi	Temp, 'K'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 
_MStop_1:
 lds	Temp, AdaptStep
 inc	Temp 
 sts	AdaptStep, Temp
 ret


//End of File//