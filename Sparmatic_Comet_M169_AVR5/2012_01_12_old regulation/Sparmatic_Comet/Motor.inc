
;--------------------------------------------------
Adaptation:
 sbis	Status0, _Adapt
 ret

 sbis	Status0, _MotDir					;check only, if valve gets closed
 rjmp	_Adapt3

 lds	Temp, RFLXCTL
 lds	TempH, RFLXCTH

 cpi	Temp, 20							;get motor current with no load on position n
 cpc	TempH, Null
 brne	_Adapt1

 lds	Temp3, MotorCurrent
 sts	FreeMotorCurrent, Temp3
 lds	Temp3, DisplayBuffer1+11			;set bargraph point 1, freewheeling current measured
 ori	Temp3, 0b00000001
 sts	DisplayBuffer1+11, Temp3
 lds	Temp3, DisplayBuffer2+11						 
 ori	Temp3, 0b00000001
 sts	DisplayBuffer2+11, Temp3
 
_Adapt1:
 sbis	Status0, _MotDir					;check only, if valve gets closed
 rjmp	_Adapt3
 
 lds	Temp3, MotorCurrent
 lds	Temp4, FreeMotorCurrent
 sub	Temp3, Temp4
 brcc	_Adapt2
 clr	Temp3

_Adapt2:
 cpi	Temp3, 10							;threshold current over freewheeling current, to detect valve touch
 brcs	_Adapt3

 mov	Temp3, Temp
 mov	Temp4, TempH
 adiw	Temp4:Temp3, 2

 sts	OutLimitL, Temp3					;store new valve out limit
 sts	OutLimitH, Temp4

 lds	Temp3, ValveTopL					;if Valve top value is not on default, skip
 lds	Temp4, ValveTopH
 cpi	Temp3, 10							
 cpc	Temp4, Null
 brne	_Adapt3

 sts	ValveTopL, Temp						;save new valve top value
 sts	ValveTopH, TempH

 lds	Temp, DisplayBuffer1+6				;set bargraph point 2, touch current measured
 ori	Temp, 0b00000001
 sts	DisplayBuffer1+6, Temp
 lds	Temp, DisplayBuffer2+6					
 ori	Temp, 0b00000001
 sts	DisplayBuffer2+6, Temp
 

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
 breq	_AdpEnd 
 ret

_AdpOpenValve:
 ldi	Temp, $00
 sts	PositionL, Temp
 ldi	Temp, $10
 sts	PositionH, Temp
 sbi 	Status0, _MotOn	
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
 ret

_WaitADP_OK:
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
 ret


_AdpCloseValve:
 ldi	Temp, $00
 sts	PositionL, Temp
 ldi	Temp, $20
 sts	PositionH, Temp
 sbi 	Status0, _MotOn	
 inc	TempH
 sts	AdaptStep, TempH
 ret

_AdpEnd:
 lds	Temp, RFLXCTL
 lds	TempH, RFLXCTH
 subi	Temp, 10						;store margin to fully closed valve as 'ValveBottom'
 sbc	TempH, Null 
 sts	ValveBOTL, Temp					;store valve bottom value
 sts	ValveBOTH, TempH
 cbi	Status0, _Adapt
 sts	AdaptStep, Null
 sts	PositionL, Temp
 sts	PositionH, TempH
 sbi 	Status0, _MotOn	
 lds	Temp, DisplayBuffer1+11					;clear bargraph points 1 and 2
 andi	Temp, 0b11111110
 sts	DisplayBuffer1+11, Temp
 lds	Temp, DisplayBuffer2+11					;clear bargraph points 1 and 2
 andi	Temp, 0b11111110
 sts	DisplayBuffer2+11, Temp
  
 lds	Temp, DisplayBuffer1+6
 andi	Temp, 0b11111110
 sts	DisplayBuffer1+6, Temp
 lds	Temp, DisplayBuffer2+6
 andi	Temp, 0b11111110
 sts	DisplayBuffer2+6, Temp
 
 ldi	Temp, 3
 sts	ADCPrescaler, Temp 
 sts	UserDisplay, Null
 sts	DisplayCT, Null
 sts	DispTimer, Null
 
 call	CalcDayOfWeek
 ori	Temp, $80
 rcall	ClearWeekDays
 ldi	TempH, $30
 rcall	PutWeekDay
 
 lds	Temp, DisplayBuffer1					;switch on hour bar
 ori	Temp, 0b00000001	
 sts	DisplayBuffer1, Temp
 lds	Temp, DisplayBuffer2
 ori	Temp, 0b00000001	
 sts	DisplayBuffer2, Temp

 ret
 


;--------------------------------------------------
CheckPosition:
 sbis	Status0, _MotOn									;if motor is off, exit
 ret
 
 ldi	Temp, (1<<LCDEN) | (1<<LCDIE) | (1<<LCDAB)		;enable LCD-Frame Interrupt, used for upcounting Motor timeout
 sts	LCDCRA, Temp
 ldi	ActiveCT, 3
 sbi	BacklightPort, Backlight_On

 lds	Temp, MotTimeOut
 sbrc	Temp, _PosEqual
 rjmp	_CPosE

 lds	Temp, PositionL
 lds	TempH, PositionH
 lds	Temp3, RFLXCTL
 lds	Temp4, RFLXCTH

 cpi	TempH, $10										;position for fully opened valve
 breq	_PosOpen
 cpi	TempH, $20										;position for fully closed valve
 breq	_PosClose
 
 cp		Temp3, Temp										;if real position is lower than set, close valve
 cpc	Temp4, TempH
 brcs	_PosClose

 cp		Temp, Temp3										;if real position is higher than set, open valve
 cpc	TempH, Temp4
 brcs	_PosOpen
 
 ret
 ;rjmp	_CPosE


_PosOpen:
 sbic	Status0, _MotRun								;execute only if motor hardware was stopped before
 ret
 sbi	LightSensPort, LightSens_LED
 cbi	Status0, _MotDir
 sts	MotTimeOut, Null
 ret

_PosClose:
 sbic	Status0, _MotRun								;execute only if motor hardware was stopped before
 ret
 sbi	LightSensPort, LightSens_LED		
 sbi	Status0, _MotDir
 sts	MotTimeOut, Null
 ret

_CPosE:
 clr	Temp											;set flag, that the desired position was reached
 sbr	Temp, (1<<_PosReached)
 sts	MotTimeOut, Temp

 ;sbi	DebugPort, Debug2

 rjmp	_MotorStop





;------------------------------------------------------------------------------
MotorControl:
 lds	Temp, MotTimeOut					
 andi	Temp, 0b00111111
 cpi	Temp, 20										;timeout value to completely shut down motor hardware
 brcs	_MC1
 cbi	Status0, _MotRun					
 cbi	LightSensPort, LightSens_LED					;switch off reflex sensor
 
 sbic	Status0, _MotDir								;if timeout and open direction, check for upper limit
 ret
 
 lds	Temp, MotTimeOut
 sbrc	Temp, _PosReached								;if position was reached before stop, don´t reset ReflexCounter
 ret

 sts	RFLXCTL, Null									;if valve is on upper limit, clear position counter
 sts	RFLXCTH, Null
 ret

_MC1: 
 cpi	Temp, 10										;timeout value to shut down motor
 brcc	_MotorStop

 sbis	Status0, _MotOn									;exit, if motor has stopped
 ret

 sbi	Status0, _MotRun

 sbis	Status0, _MotDir								;dependent on direction flag open or close valve
 rjmp	_Motor_Open										;flag is clear: open, flag is set: close

_MotorClose:
 lds	Temp, RFLXCTL
 lds	TempH, RFLXCTH
 lds	Temp3, OutLimitL
 lds	Temp4, OutLimitH
 cp		Temp, Temp3
 cpc	TempH, Temp4
 brcc	_MotorStop
 cbi	MotorDDR, Motor_Open
 cbi	MotorPort, Motor_Open
 nop
 nop
 sbi	MotorDDR, Motor_Close
 sbi	MotorPort, Motor_Close
 ret

_Motor_Open:
 cbi	MotorDDR, Motor_Close
 cbi	MotorPort, Motor_Close
 nop
 nop
 sbi	MotorDDR, Motor_Open
 sbi	MotorPort, Motor_Open
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
