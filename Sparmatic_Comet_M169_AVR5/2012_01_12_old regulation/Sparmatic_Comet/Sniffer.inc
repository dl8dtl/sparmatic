;------------------------------------
DebugSniffer:
 lds	TempH, DebugAddress
 ldi	XL, low(DebugBuffer)
 ldi	XH, high(DebugBuffer)
 add	XL, TempH
 adc	XH, Null
 ld		Temp, X
 ldi	Temp3, $02					;ASC
 rcall	Sniffer
 lds	TempH, DebugAddress
 inc	TempH
 sts	DebugAddress, TempH
 ret


 ;------------------------------------
Sniffer:
 rcall	SnifferStart
 swap	Temp3
 push	TempH
 ldi	TempH, 4
 
_SN1:
 rol	Temp3
 brcs	_SN11
 rcall	SnifferLow
 rjmp	_SN12

_SN11:
 rcall	SnifferHigh

_SN12:
 dec	TempH
 brne	_SN1

 ldi	Temp3, 8
 pop	TempH

_SN2:
 rol	TempH
 brcs	_SN21
 rcall	SnifferLow
 rjmp	_SN22

_SN21:
 rcall	SnifferHigh
 
_SN22:
 dec	Temp3
 brne	_SN2
 
_SN3:
 ldi	Temp3, 8

_SN31:
 rol	Temp
 brcs	_SN32
 rcall	SnifferLow
 rjmp	_SN33

_SN32:
 rcall	SnifferHigh
 
_SN33:
 dec	Temp3
 brne	_SN31
 ret


;----------------------------
SnifferStart:
 push	Temp
 push	TempH 
 cli
 rcall	SnifferToggle
 rcall	SnifferToggle
 rcall	SnifferToggle
 sei
 andi	Temp3, 0b00001111
 pop	TempH
 pop	Temp 
 ret

SnifferHigh:
 push	Temp
 push	TempH
 cli
 rcall	SnifferToggle
 rcall	SnifferToggle
 sei
 pop	TempH
 pop	Temp
 ret

SnifferLow:
 push	Temp
 push	TempH 
 cli
 rcall	SnifferToggle
 sei
 pop	TempH
 pop	Temp
 ret

SnifferToggle:							;insert or delete 'nop' to tweak to >=1µs pulse phase interval
 sbi	DebugPin, SnifPin
 ret

;------------------------------------------------------------------------
FullDebug:
 ldi	Temp, 'R'
 sts	DebugBuffer+0, Temp
 ldi	Temp, 'F'
 sts	DebugBuffer+1, Temp
 ldi	Temp, 'X'
 sts	DebugBuffer+2, Temp
 
 lds	Temp, RFLXCTH
 call	DivHex
 sts	DebugBuffer+4, Di1
 sts	DebugBuffer+5, Di0

 lds	Temp, RFLXCTL
 call	DivHex
 sts	DebugBuffer+6, Di1
 sts	DebugBuffer+7, Di0
 
 ret


 ldi	Temp, 'P'
 sts	DebugBuffer+10, Temp
 ldi	Temp, 'O'
 sts	DebugBuffer+11, Temp
 ldi	Temp, 'S'
 sts	DebugBuffer+12, Temp
 
 lds	Temp, PositionH
 call	DivHex
 sts	DebugBuffer+14, Di1
 sts	DebugBuffer+15, Di0

 lds	Temp, PositionL
 call	DivHex
 sts	DebugBuffer+16, Di1
 sts	DebugBuffer+17, Di0

 ldi	Temp, 'V'
 sts	DebugBuffer+20, Temp
 ldi	Temp, 'T'
 sts	DebugBuffer+21, Temp
 ldi	Temp, 'P'
 sts	DebugBuffer+22, Temp
 
 lds	Temp, ValveTOPH
 call	DivHex
 sts	DebugBuffer+24, Di1
 sts	DebugBuffer+25, Di0

 lds	Temp, ValveTOPL
 call	DivHex
 sts	DebugBuffer+26, Di1
 sts	DebugBuffer+27, Di0

 ldi	Temp, 'V'
 sts	DebugBuffer+30, Temp
 ldi	Temp, 'B'
 sts	DebugBuffer+31, Temp
 ldi	Temp, 'T'
 sts	DebugBuffer+32, Temp
 
 lds	Temp, ValveBOTH
 call	DivHex
 sts	DebugBuffer+34, Di1
 sts	DebugBuffer+35, Di0

 lds	Temp, ValveBOTL
 call	DivHex
 sts	DebugBuffer+36, Di1
 sts	DebugBuffer+37, Di0
 
 
 ldi	Temp, 'B'
 sts	DebugBuffer+40, Temp
 ldi	Temp, 'D'
 sts	DebugBuffer+41, Temp
 ldi	Temp, 'B'
 sts	DebugBuffer+42, Temp
 
 mov	Temp, ButtonDebounce
 call	DivHex
 sts	DebugBuffer+44, Di1
 sts	DebugBuffer+45, Di0

 ldi	Temp, 'O'
 sts	DebugBuffer+50, Temp
 ldi	Temp, 'B'
 sts	DebugBuffer+51, Temp
 ldi	Temp, 'T'
 sts	DebugBuffer+52, Temp
 
 lds	Temp, OldButtons
 call	DivHex
 sts	DebugBuffer+54, Di1
 sts	DebugBuffer+55, Di0
 
 
 
 ldi	Temp, 'O'
 sts	DebugBuffer+60, Temp
 ldi	Temp, 'L'
 sts	DebugBuffer+61, Temp
 ldi	Temp, 'T'
 sts	DebugBuffer+62, Temp
 
 lds	Temp, OutLimitH
 call	DivHex
 sts	DebugBuffer+64, Di1
 sts	DebugBuffer+65, Di0

 lds	Temp, OutLimitL
 call	DivHex
 sts	DebugBuffer+66, Di1
 sts	DebugBuffer+67, Di0


 ldi	Temp, 'M'
 sts	DebugBuffer+70, Temp
 ldi	Temp, 'C'
 sts	DebugBuffer+71, Temp
 ldi	Temp, 'U'
 sts	DebugBuffer+72, Temp
 
 lds	Temp, MotorCurrent
 call	DivHex
 sts	DebugBuffer+74, Di1
 sts	DebugBuffer+75, Di0

 lds	Temp, FreeMotorCurrent
 call	DivHex
 sts	DebugBuffer+77, Di1
 sts	DebugBuffer+78, Di0

 lds	Temp, Hours
 clr	TempH
 call	DivDec
 sts	DebugBuffer+120, Di1
 sts	DebugBuffer+121, Di0
 ldi	Temp,':'
 sts	DebugBuffer+122, Temp
 lds	Temp, Minutes
 clr	TempH
 call	DivDec
 sts	DebugBuffer+123, Di1
 sts	DebugBuffer+124, Di0
 ldi	Temp,':'
 sts	DebugBuffer+125, Temp
 lds	Temp, Seconds
 clr	TempH
 call	DivDec
 sts	DebugBuffer+126, Di1
 sts	DebugBuffer+127, Di0
 
 lds	Temp, TempIntL 
 lds	TempH, TempIntH
 call	DivDec
 sts	DebugBuffer+130, Di2
 sts	DebugBuffer+131, Di1
 ldi	Temp,'.'
 sts	DebugBuffer+132, Temp
 sts	DebugBuffer+133, Di0
 ldi	Temp, 7								;sniffer degree sign
 sts	DebugBuffer+134, Temp
 ldi	Temp,'C'
 sts	DebugBuffer+135, Temp
 
 lds	Temp, RegValH
 call	DivHex
 sts	DebugBuffer+138, Di1
 sts	DebugBuffer+139, Di0

 lds	Temp, RegValL
 call	DivHex
 sts	DebugBuffer+140, Di1
 sts	DebugBuffer+141, Di0
 ret