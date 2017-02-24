;--------------------------------------------------
;RealTimeClock timer, automatic DST switch
;--------------------------------------------------
Clock:
 in 	Temp, Status1
 sbrs	Temp, _SecondTick 			;execute only ,if prescaler1 is 0
 ret
 
 cli
 in 	Temp, Status1
 cbr	Temp, (1<<_SecondTick)
 out	Status1, Temp
 sei

_ClockSeconds:  
 lds	Temp, Seconds
 inc	Temp
 sts	Seconds, Temp
 cpi	Temp, 60
 breq	_ClockSec1
 rjmp	_ClockEnd		

_ClockSec1:
 sts	Seconds, Null
 lds	Temp, Minutes
 inc	Temp
 sts	Minutes, Temp

;------------------------------
;fine tune for inaccurate crystals

/* lds	Temp, TimeAdjust
 dec	Temp
 sts	TimeAdjust, Temp
 brne	_Clock1
 ldi	Temp, 5
 sts	PSC1, Temp					;adjust value for prescaler1 interval (default 4)
 ldi	Temp, 35					;TimeAdjust interval counter
 sts	TimeAdjust, Temp*/

;-------------------------------

_Clock1:
 ;lds	Temp, WinOpCDMin			;decrement open windows counter, if zero, resume to normal mode
 ;cpi	Temp, 0
 ;breq	_ClockMinutes
 ;dec	Temp
 ;sts	WinOpCDMin, Temp
 ;brne	_ClockMinutes
 ;lds	Temp, TempMode
 ;cbr	Temp, (1<<1)
 ;sts	TempMode, Temp
 ;call	_WindowClosed

_ClockMinutes:
 lds	Temp, Minutes
 cpi	Temp, 60
 breq	_ClockMin1
 rjmp	_ClockEnd

_ClockMin1:
 sts	Minutes, Null
 lds	Temp, Hours
 inc	Temp
 sts	Hours, Temp

 lds	Temp, WDays					;Friday 11:00:00
 cpi	Temp, 4
 brne	_ClockMin2
 lds	Temp, Hours
 cpi	Temp, 11
 brne	_ClockMin2
 cli
 in		Temp, Status1				;start Descale function
 sbr	Temp, (1<<_Descale)
 out	Status1, Temp
 sei

_ClockMin2:
 in		Temp, Status1
 sbrc	Temp, _DST_OnOff			;if DST toggle has already done, skip DST check
 rjmp	_ClockHours

 lds	Temp, Days					;last week on month?
 cpi	Temp, 24
 brcs	_ClockHours

 lds	Temp, WDays					;last sunday on month?
 cpi	Temp, 6
 brne	_ClockHours

 lds	Temp, Months
 cpi	Temp, 9						;Oktober?
 breq	_Summer_Winter
 cpi	Temp, 2						;March?
 breq	_Winter_Summer
 rjmp	_ClockHours

_Summer_Winter:
 lds	Temp, Hours
 cpi	Temp, 3
 brne	_ClockHours

 dec	Temp
 sts	Hours, Temp
 rjmp	_SetDST

_Winter_Summer:
 lds	Temp, Hours
 cpi	Temp, 2
 brne	_ClockHours

 inc	Temp
 sts	Hours, Temp

_SetDST:
 cli
 in		Temp, Status1
 sbr	Temp, (1<<_DST_OnOff)
 out	Status1, Temp
 sei

_ClockHours:
 lds	Temp, Hours
 cpi	Temp, 24
 breq	_ClockHour1
 rjmp	_ClockEnd

_ClockHour1:
 cli
 in		Temp, Status1
 cbr	Temp, (1<<_DST_onoff)
 out	Status1, Temp
 sei

 sts	Hours, Null
 lds	Temp, Days
 inc	Temp
 sts	Days, Temp

 lds	Temp, WDays
 inc	Temp
 sts	WDays, Temp
 cpi	Temp, 7
 brne	_ClockDays
 clr	Temp
 sts	WDays, Temp

_ClockDays:
 ori	Temp, $80
 call	ClearWeekDays
 ldi	TempH, $30
 call	PutWeekDay 

 in		Temp, Status1
 sbrs	Temp, _AutoManu
 call	Show_DailyBargraph
 
 rcall	MonthLastDay

_ClockDay1:
 lds	Temp, Days
 cp 	Temp, TempH
 brne	_ClockEnd

 sts	Days, Null
 lds	Temp, Months
 inc	Temp
 sts	Months, Temp

_ClockMonths:
 lds	Temp, Months
 cpi	Temp, 12
 brne	_ClockEnd

 sts	Months, Null
 lds	Temp, Years
 inc	Temp
 sts	Years, Temp

_ClockEnd:
 in		Temp, Status1
 sbrs	Temp, _Descale
 rjmp	_ClockEnd1

 sbic	Status0, _Adapt
 rjmp	_ClockEnd1
 
 call	ClearColon
 call	ClearPoint
 call	ClearBargraph
 call	StartMain


_ClockEnd1:
 ret


;--------------------------------------------------------------------------------------------------
;w=(d+MonthGauss+y+(y/4)+(c/4)+5*c) mod7 // Sunday=0...Saturday=6
;d=1..31 // y=Year ..nn // c= Year nn..
;Jan. + Feb. -> y=y-1
CalcDayOfWeek:
 lds	Temp, Months
 ldi	ZL, low(MonthGauss*2)
 ldi	ZH, high(MonthGauss*2)
 add	ZL, Temp
 adc	ZH, Null
 lds	Di0, Days
 inc	Di0
 lpm	Di1, Z					;load month value
 lds	Di2, Years
 lds	Temp, Months			;if months are 0 or 1 (January or February), decrement y
 cpi	Temp, 2
 brcc	_CDOW1
 dec	Di2
 
_CDOW1:
 add	Di0, Di1				;add GaussMonths
 add	Di0, Di2 				;add Year
 lsr	Di2						;y/4
 lsr	Di2
 ldi	Temp, 20
 lsr	Temp					;c/4
 lsr	Temp
 
 add	Di0, Di2
 add	Di0, Temp
 ldi	Temp, 100				;c*5
 add	Di0, Temp

 ldi	Temp, 7

_CDOWLoop:
 cp 	Di0, Temp				;modulo 7
 brcs	_CDOW_End
 sub	Di0, Temp
 rjmp	_CDOWLoop
 
_CDOW_End:
 mov	Temp, Di0
 subi	Temp, 1						
 brcc	_CDOW_End1
 ldi	Temp, 6

_CDOW_End1: 
 sts	WDays, Temp
 ret


;--------------------------------------------------------------------------------
;check for last Day of a month
;input: none / output: TempH, last day in a month
MonthLastDay:
 lds	Temp, Months
 ldi	ZL, low(MonthDayNo*2)
 ldi	ZH, high(MonthDayNo*2)
 add	ZL, Temp
 lpm 	TempH, Z
 
 lds	Temp, Years
 andi	Temp, 0b00000011			;check for leap year
 brne	_MLD1
 lds	Temp, Months
 cpi	Temp, 1						;if month is february, add 1 day
 brne	_MLD1
 inc	TempH

_MLD1:
 ret




MonthGauss:
.db 28,31,2,5,7,10,12,15,18,20,23,25
;,Januar,Februar,März,April,Mai,Juni,Juli,August,September,Oktober,November,Dezember

MonthDayNo:
.db 31,28,31,30,31,30,31,31,30,31,30,31



//End of file//