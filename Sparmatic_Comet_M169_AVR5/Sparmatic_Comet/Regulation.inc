;--------------------------------------------------
; adaptive regulation function
Regulate:
 lds	Temp, TempIntL						;NTC temperature
 lds	TempH, TempIntH
 lds	Di0, TempIntOldL					;store current temperature as old for next run
 lds	Di1, TempIntOldH
 sts	TempIntOldL, Temp
 sts	TempIntOldH, TempH
 lds	Di2, DeltaTemp1						;mov previous Delta1 to Delta2
 sts	DeltaTemp2, Di2
 sub	Temp, Di0							;get new Delta1
 sbc	TempH, Di1
 sts	DeltaTemp1, Temp
 
 ldi	Temp3, TStatic
 breq	_Reg1								;CurrentTemp = OldTemp 
 ldi	Temp3, TFall
 brmi	_Reg1								;CurrentTemp is lower than OldTemp
 ldi	Temp3, TRise						;CurrentTemp is higher than OldTemp

_Reg1:
 add	Temp, Di2							;add both Delta values
 brpl	_RegPl								;if positive, execute / if negative, do 1´s complement
 neg	Temp

_RegPl:
 cpi	Temp, 0								;if adding both deltas equals to 0, add 1
 brne	_RegPl1
 inc	Temp

_RegPl1:
 mov	Di0, Temp 
 ldi 	Temp, 4								;limit gradient multiplicand
 cp		Di0, Temp
 brcs	_Reg11
 mov	Di0, Temp

_Reg11:
 lds	Temp, TempIntL						;NTC temperature
 lds	TempH, TempIntH
 lds	Di1, SetTempL						;current set temperature
 lds	Di2, SetTempH

 sub	Temp, Di1							
 sbc	TempH, Di2 
 brmi	_TempLowerThanSet
 

 ldi	Temp4, FuzzAbove
 cpi	Temp, 15
 cpc	TempH, Null
 brcc	_Fuzzy
 
 ldi	Temp4, FuzzHot
 cpi	Temp, 6
 cpc	TempH, Null
 brcc	_Fuzzy
 
 ldi	Temp4, FuzzWarm
 cpi	Temp, 2
 cpc	TempH, Null
 brcc	_Fuzzy
 rjmp	_FuzzOK

_TempLowerThanSet:
 com	TempH
 neg	Temp
 
 ldi	Temp4, FuzzBelow
 cpi	Temp, 15
 cpc	TempH, Null
 brcc	_Fuzzy
   
 ldi	Temp4, FuzzCold
 cpi	Temp, 6
 cpc	TempH, Null
 brcc	_Fuzzy
 
 ldi	Temp4, FuzzCool
 cpi	Temp, 2
 cpc	TempH, Null
 brcc	_Fuzzy

_FuzzOK: 
 ldi	Temp4, FuzzOK

_Fuzzy:
 add	Temp4, Temp3
 sts	FuzzyVal, Temp4

 ldi	ZL, low (_FuzzyJump)
 ldi	ZH, high (_FuzzyJump)
 add	ZL, Temp4
 adc	ZH, Null
 ijmp
 
_FuzzyJump:
 rjmp	_OK_Static				; 0
 rjmp	_OK_Rise				; 1
 rjmp	_OK_Fall				; 2
 rjmp	_NoRegulation			;dummy  3
 rjmp	_Warm_Static			; 4
 rjmp	_Warm_Rise				; 5
 rjmp	_Warm_Fall				; 6
 rjmp	_NoRegulation			;dummy  7
 rjmp	_HOT_Static				; 8
 rjmp	_HOT_Rise				; 9
 rjmp	_HOT_Fall				;10
 rjmp	_NoRegulation			;dummy 11
 rjmp	_Cool_Static			;12
 rjmp	_Cool_Rise				;13
 rjmp	_Cool_Fall				;14
 rjmp	_NoRegulation			;dummy 15
 rjmp	_Cold_Static			;16
 rjmp	_Cold_Rise				;17
 rjmp	_Cold_Fall				;18
 rjmp	_NoRegulation			;dummy 19
 rjmp	_Above_Static			;20
 rjmp	_Above_Rise				;21
 rjmp	_Above_Fall				;22
 rjmp	_NoRegulation			;dummy 23
 rjmp	_Below_Static			;24
 rjmp	_Below_Rise				;25
 rjmp	_Below_Fall				;26
 rjmp	_NoRegulation			;dummy 27

 

_OK_Static:
 sts	RegWay, Null
 ret

_OK_Rise:
 sts	RegWay, Null
 ret

_OK_Fall:
 sts	RegWay, Null
 ret

_NoRegulation:
 ret

_Warm_Static:
 ldi	Temp3, 3
 rjmp	_RegCloseValve
 ret

_Warm_Rise:
 ldi	Temp3, 10
 mul	Temp3, Di0
 mov	Temp3, R0
 rjmp	_RegCloseValve

_Warm_Fall:
 ldi	Temp3, 10
 mul	Temp3, Di0
 mov	Temp3, R0
 rjmp	_RegOpenValve

_HOT_Static:
 ldi	Temp3, 10
 rjmp	_RegCloseValve

_HOT_Rise:
 ldi	Temp3, 20
 mul	Temp3, Di0
 mov	Temp3, R0
 rjmp	_RegCloseValve

_Hot_Fall:
 sts	RegWay, Null
 ret

_Cool_Static:
 ldi	Temp3, 3
 rjmp	_RegOpenValve

_Cool_Rise:
 ldi	Temp3, 10
 mul	Temp3, Di0
 mov	Temp3, R0
 rjmp	_RegCloseValve

_Cool_Fall:
 ldi	Temp3, 10
 mul	Temp3, Di0
 mov	Temp3, R0
 rjmp	_RegOpenValve

_Cold_Static:
 ldi	Temp3, 10
 rjmp	_RegOpenValve

_Cold_Rise:
 sts	RegWay, Null
 ret
 
_Cold_Fall:
 ldi	Temp3, 20
 mul	Temp3, Di0
 mov	Temp3, R0
 rjmp	_RegOpenValve
 

_Above_Static:
 ldi	Temp3, 50
 rjmp	_RegCloseValve
 ret

_Above_Rise:
 ldi	Temp3, 100
 rjmp	_RegCloseValve
 ret

_Above_Fall:
 ldi	Temp3, 50
 rjmp	_RegCloseValve
 ret

_Below_Static:
 ldi	Temp3, 50
 rjmp	_RegOpenValve
 ret

_Below_Rise:
 ldi	Temp3, 50
 rjmp	_RegOpenValve
 ret

_Below_Fall:
 ldi	Temp3, 100
 rjmp	_RegOpenValve
 ret


_RegCloseValve:
 sts	RegWay, Null
 lds	Temp, PositionL
 lds	TempH, PositionH
 sub	Temp, Temp3
 sbc	TempH, Null
 sts	PositionL, Temp
 sts	PositionH, TempH
 brcc	_RCV0
 
 sts	PositionL, Null
 sts	PositionH, Null

_RCV0:  
 lds	Temp, MotTimeOut					;ignore closing the valve, if valve is fully closed
 sbrc	Temp, _BotLimit
 ret

 sbi	Status0, _MotDir					;close valve
 rjmp	_ROV1
 

_RegOpenValve:								
 sts	RegWay, Null
 lds	Temp, PositionL						;limit position change to ValveTop position
 lds	TempH, PositionH
 add	Temp, Temp3
 adc	TempH, Null
 sts	PositionL, Temp
 sts	PositionH, TempH
 lds	Di0, ValveTopL
 lds	Di1, ValveTopH						;ignore opening the valve, if valve top limit is reached
 sub	Di0, Temp
 sbc	Di1, TempH
 brpl	_ROV0

 lds	Temp, ValveTopL						;load max value to position
 lds	TempH, ValveTopH
 sts	PositionL, Temp
 sts	PositionH, TempH
 
 neg	Di0
 sub	Temp3, Di0
 brcc	_ROV0
 ret


_ROV0: 
 lds	Temp, MotTimeOut
 sbrc	Temp, _TopLimit
 ret
 
 cbi	Status0, _MotDir					;open valve

_ROV1:
 sts	RegWay, Temp3
 
 sbic	Status0, _MotRun					;do not store new motor position, if motor is running
 ret
 
 sbi 	Status0, _MotOn	
 lds	Temp, MotTimeOut
 andi	Temp, 0b11000000
 sts	MotTimeOut, Temp
 sts	RFLXCTL, Temp3
 sts	RFLXCTH, Null
 ret

