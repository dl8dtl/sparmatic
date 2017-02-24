BargraphTest:
 lds	Temp, Test
 ldi	TempH, $30
 rcall	PutBargraph

 lds	Temp, Test
 inc	Temp
 sts	Test, Temp
 andi	Temp, $7F
 cpi	Temp, 24
 brne	_BGT1
 lds	Temp, Test
 andi	Temp, $80
 ldi	TempH, $80
 eor	Temp, TempH
 sts	Test, Temp
 
_BGT1:
 ret



/* ;--------------------------------------------------
Show_NTC_ADC:
 lds	Temp, NTCIntL
 lds	TempH, NTCIntH
 rcall	DivDec

 ldi	TempH, 0
 mov	Temp, Di2
 rcall	PutCharacter
 ldi	TempH, 1
 mov	Temp, Di1
 rcall	PutCharacter
 ldi	TempH, 2
 mov	Temp, Di0
 rcall	PutCharacter
 ldi	TempH, 3
 ldi	Temp, '?'					;space
 rcall	PutCharacter


 lds	Temp, LCDDR7				;clr point between segment 1 and 2
 andi	Temp, 0b01111111
 sts	LCDDR7, Temp
 ret
*/


/* ;--------------------------------------------------
Show_Motor:
 lds	Temp, MotTimeout
 rcall	DivHex

 ldi	TempH, 0
 mov	Temp, Di1
 rcall	PutCharacter
 ldi	TempH, 1
 mov	Temp, Di0
 rcall	PutCharacter

 lds	Temp, RFLXCTL
 rcall	DivHex

 ldi	TempH, 2
 mov	Temp, Di1
 rcall	PutCharacter
 ldi	TempH, 3
 mov	Temp, Di0
 rcall	PutCharacter
 ret*/

/*  ;--------------------------------------------------
Show_MotorCurrent:
 lds	Temp, MotTimeout
 rcall	DivHex
 ldi	TempH, 0
 mov	Temp, Di1
 rcall	PutCharacter
 ldi	TempH, 1
 mov	Temp, Di0
 rcall	PutCharacter

 lds	Temp, MotorCurrent
 rcall	DivHex

 ldi	TempH, 2
 mov	Temp, Di1
 rcall	PutCharacter
 ldi	TempH, 3
 mov	Temp, Di0
 rcall	PutCharacter
 ret

*/