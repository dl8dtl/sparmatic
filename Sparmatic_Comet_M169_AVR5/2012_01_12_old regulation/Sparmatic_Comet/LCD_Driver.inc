;--------------------------------------------------
Show_TimerSetBar:
 ld		Temp, X
 push	Temp								;save current set time
 rcall	Clear_Bargraph
 ldi	Temp3, 4							;number of inhouse/offhouse timers
 lds	XL, BarBaseL
 lds	XH, BarBaseH

S_TSB0: 
 ldi	Temp4, 0
 ld		Temp, X+ 
 rcall	TimerHour2Bin

S_TSB1: 
 cp 	Temp, Temp4
 breq	_BarStart
 inc	Temp4
 cpi	Temp4, 24
 breq	_BarEnd
 rjmp	S_TSB1

_BarStart: 
 ld		Temp, -X							;set Barpoint only, if off time is later then on time
 adiw	XH:XL, 1
 ld		TempH, X
 cp		Temp, TempH
 brcc	_BarEnd1
 
 mov	Temp, Temp4
 ori	Temp, $80
 ldi	TempH, $30
 rcall	PutBargraph

 ld		Temp, X+ 
 dec	Temp
 rcall	TimerHour2Bin
 inc	Temp
 
S_TSB2: 
 cp 	Temp, Temp4
 breq	_BarEnd

 push	Temp
 mov	Temp, Temp4
 ori	Temp, $80
 ldi	TempH, $30
 rcall	PutBargraph
 pop	Temp

 inc	Temp4
 cpi	Temp4, 24
 brne	S_TSB2

_BarEnd: 
 dec	Temp3
 brne	S_TSB0 

 pop	Temp									;restore current set time and blink
 rcall	TimerHour2Bin
 push	Temp
 ldi	TempH, $20
 rcall	PutBargraph
 pop	Temp
 ori	Temp, $80
 ldi	TempH, $10
 rcall	PutBargraph
 ret

_BarEnd1:
 adiw	XH:XL, 1
 rjmp	_BarEnd



  /*
 lds	TempH, MenuLow
 sbrc	TempH, 0
 rjmp	_BST2

 push	Temp
 rcall	TimerHour2Bin
 push	Temp
 ldi	TempH, $20
 rcall	PutBargraph
 pop	Temp
 ori	Temp, $80
 ldi	TempH, $10
 rcall	PutBargraph
 pop	Temp
 
_BST2:*/ 








/*
 ldi	Temp4, 0
 lds	Temp, Status1
 cbr	Temp, (1<<ToggleFlag)
 sts	Status1, Temp
    
_Next_BarCounter:
 ldi	TempH, 4
 lds	XL, BarBaseL
 lds	XH, BarBaseH
 clr	Temp3
 rcall	_STSB_CheckLoop				;check for on time
 cpi	Temp3, 1
 breq	_STSB_OnTime				
 
 ldi	TempH, 4
 lds	XL, BarBaseL
 lds	XH, BarBaseH
 adiw	XH:XL, 1					;check for off time
 clr	Temp3
 rcall	_STSB_CheckLoop
 cpi	Temp3, 1
 breq	_STSB_OffTime

 rcall	Write_LastBarState


_STSB_CheckLoop:
 ld		Temp, X						
 cp		Temp, Temp4
 brne	_STSB_OnLoop1
 ldi	Temp3, 1

_STSB_OnLoop1:
 adiw	XH:XL, 2
 dec	TempH
 brne	_STSB_OnLoop 
 ret


_STSB_OnTime:
 ldi	TempH, 4
 lds	XL, BarBaseL
 lds	XH, BarBaseH
 adiw	XH:XL, 1					;check for off time
 clr	Temp3
 rcall	_STSB_CheckLoop
 cpi	Temp3, 1
 brne	_STSB_OnTime1				;if on and off time are qual, simply increment counter
 rjmp	_Inc_BarCounter



_STSB_OnTime1:







 
_STSB_OffTime:











_NT1:
 cp		Temp, Temp4
 breq	_BarStart
 inc	Temp4
 cpi	Temp4, 24
 brne	_NT1
 rjmp	_NextTimer2
 
_BarStart:
 mov	Temp, Temp4
 ori	Temp, $80
 ldi	TempH, $30
 rcall	PutBargraph
 
 ld		Temp, X+					;offhouse timer
 rcall	TimerHour2Bin

_NT2:
 mov	TempH, Temp4
 dec	TempH
 cp		Temp, TempH
 breq	_NextTimer2

 push	Temp
 mov	Temp, Temp4
 ori	Temp, $80
 ldi	TempH, $30
 rcall	PutBargraph
 pop	Temp
 inc	Temp4
 cpi	Temp4, 24
 brne	_NT2
  
_NextTimer2:
 dec	Temp3
 brne	_NextTimer
 ret


_Inc_BarCounter:
 inc	Temp4
 mov	Temp, Temp4
 rcall	DivDez
 ldi	Temp, '6'
 cp		Di0, Temp
 brne	_I_BC1
 subi	TempH, -4

_I_BC1:
 cpi	TempH, 240
 brne	_Next_BarCounter
 ret*/



/*
 ld	Temp, X
 rcall	TimerHour2Bin
 lds	TempH, OldBarBlink
 sts	OldBarBlink, Temp
 cp		Temp, TempH
 breq	_S_TSB1
 push	Temp
 mov	Temp, TempH
 ldi	TempH, $30
 rcall	PutBargraph
 pop	Temp

_S_TSB1:
 push	Temp
 ldi	TempH, $20
 rcall	PutBargraph
 pop	Temp
 ori	Temp, $80
 ldi	TempH, $10
 rcall	PutBargraph
 */
 
 ret


 ;--------------------------------------------------
Clear_BarGraph:
  sts	DisplayBuffer1+1, Null
  sts	DisplayBuffer2+1, Null
  sts	DisplayBuffer1+6, Null
  sts	DisplayBuffer2+6, Null
  lds	Temp, DisplayBuffer1+11
  andi	Temp, 0b10000000
  sts	DisplayBuffer1+11, Temp
  lds	Temp, DisplayBuffer2+11
  andi	Temp, 0b10000000
  sts	DisplayBuffer2+11, Temp
  lds	Temp, DisplayBuffer1+16
  andi	Temp, 0b10111111
  sts	DisplayBuffer1+16, Temp
  lds	Temp, DisplayBuffer2+16
  andi	Temp, 0b10111111
  sts	DisplayBuffer2+16, Temp
  ret



;--------------------------------------------------
Clear_Screen:
 ldi	XL, low (DisplayBuffer1)
 ldi	XH, high (DisplayBuffer1)
 ldi	TempH, 40

_CSLoop:
 st		X+, Null
 dec	TempH
 brne	_CSLoop
 ret

;--------------------------------------------------
Show_Current_Temperature:
 sbic	Status0, _MenuOn			;don´t show current temperature in menu mode
 ret
 
 lds	Temp, UserDisplay			;in normal display mode show current temperature
 tst	Temp
 breq	_SH_CTe0

 cpi	Temp, 5						;update display only once
 breq	_SH_CST
 rjmp	_SH_CTeE

_SH_CST:
 lds	Temp, SetTempL
 lds	TempH, SetTempH
 
 rcall	DivHL
 mov	Temp, Di2					;blink set semperature
 ldi	TempH, $10
 rcall	PutCharacter
 mov	Temp, Di1
 rcall	PutCharacter
 mov	Temp, Di0
 rcall	PutCharacter
 ldi	Temp, '@'					;degree sign
 rcall	PutCharacter

 ldi	Temp, '?'
 ldi	TempH, $20
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'					 
 rcall	PutCharacter

 lds	Temp, DisplayBuffer1+7		;clear point between segment 1 and 2
 andi	Temp, 0b01111111
 sts	DisplayBuffer1+7, Temp
 lds	Temp, DisplayBuffer2+7		
 andi	Temp, 0b01111111
 sts	DisplayBuffer2+7, Temp
 rjmp	_SH_CTe1

_SH_CTe0:
 lds	Temp, DispTimer				;update display only once
 tst	Temp
 brne	_SH_CTeE

 lds	Temp, DisplayCT
 andi	Temp, 0b000000111
 cpi	Temp, 0
 brne	_SH_CTeE
 
 lds	Temp, TempIntL
 lds	TempH, TempIntH

 rcall	DivHL
 mov	Temp, Di2
 ldi	TempH, $30
 rcall	PutCharacter
 mov	Temp, Di1
 rcall	PutCharacter
 mov	Temp, Di0
 rcall	PutCharacter
 ldi	Temp, '@'					;degree sign
 rcall	PutCharacter

_SH_CTe1:
 lds	Temp, DisplayBuffer1+16				;clear double point between segment 1 and 2
 andi	Temp, 0b01111111
 sts	DisplayBuffer1+16, Temp
 lds	Temp, DisplayBuffer2+16				
 andi	Temp, 0b01111111
 sts	DisplayBuffer2+16, Temp
  
 lds	Temp, DisplayBuffer1+7				;set point between segment 1 and 2
 ori	Temp, 0b10000000
 sts	DisplayBuffer1+7, Temp
 lds	Temp, DisplayBuffer2+7				
 ori	Temp, 0b10000000
 sts	DisplayBuffer2+7, Temp
 
_SH_CTeE:
 ret


 ;--------------------------------------------------
Show_Current_Time:
 sbic	Status0, _MenuOn			;don´t show current time in menu mode
 ret
 
 lds	Temp, UserDisplay			;show current time in normal display mode only
 cpi	Temp, 0
 brne	_SH_CTiE

 lds	Temp, DispTimer				;update display only once
 tst	Temp
 brne	_SH_CTiE

 lds	Temp, DisplayCT
 andi	Temp, 0b000000111
 cpi	Temp, 4
 brne	_SH_CTiE
 
 lds	Temp, DisplayBuffer1+7				;clr point between segment 1 and 2
 andi	Temp, 0b01111111
 sts	DisplayBuffer1+7, Temp
 lds	Temp, DisplayBuffer2+7				
 andi	Temp, 0b01111111
 sts	DisplayBuffer2+7, Temp
 
 lds	Temp, Hours
 rcall 	DivDez
 mov	Temp, Di1

 cpi	Temp, '0'					;clear leading zero
 brne	_SH_CTi1
 ldi	Temp, '?'					;SPACE

_SH_CTi1:
 ldi	TempH, $30
 rcall	PutCharacter
 mov	Temp, Di0
 ldi	TempH, $31
 rcall	PutCharacter
 lds	Temp, Minutes
 rcall 	DivDez
 mov	Temp, Di1
 ldi	TempH, $32
 rcall	PutCharacter
 mov	Temp, Di0
 ldi	TempH, $33
 rcall	PutCharacter
 
 lds	Temp, DisplayBuffer1+16				;set colon between segment 1 and 2
 ori	Temp, 0b10000000
 sts	DisplayBuffer1+16, Temp

_SH_CTiE:
 ret




;------------------------------------------------------------------------
; write character to LCD
; Input:	Temp: Character value ASCII 
;			TempH: high nibble: Buffer number 1..3 (3 means both display buffers) / low nibble: Address 0..3 for digit position, 
;
; Output:	none
; Used:		Temp3, Temp4, ZL, ZH, XL, XH, Di0, Di1

;------------------------------------------------------------------------
PutCharacter:
 subi	Temp, 48								;start of character table in ASCII
 push	Di0
 push	Di1
 push	ZL
 push	ZH
 push	XL
 push	XH
 push	TempH									;store current address for increment after routine
 
 mov	Temp4, TempH							;save digit address
 andi	TempH, $F0
 swap	TempH
 cpi	TempH, 1
 breq	_CharacterLoop1
 cpi	TempH, 2
 breq	_CharacterLoop2
 cpi	TempH, 3
 breq	_CharacterLoop3
 rjmp	_PutCharacter_End
 
_CharacterLoop1: 
 ldi	XL, low(DisplayBuffer1)
 ldi	XH, high(DisplayBuffer1)
 rcall	_Get_Character
 rjmp	_PutCharacter_End
 
_CharacterLoop2: 
 ldi	XL, low(DisplayBuffer2)
 ldi	XH, high(DisplayBuffer2)
 rcall	_Get_Character
 rjmp	_PutCharacter_End

_CharacterLoop3: 
 ldi	XL, low(DisplayBuffer1)
 ldi	XH, high(DisplayBuffer1)
 push	Temp								;save character and address for next loop
 push	Temp4
 rcall	_Get_Character
 ldi	XL, low(DisplayBuffer2)
 ldi	XH, high(DisplayBuffer2)
 pop	Temp4								;restore character and address
 pop	Temp
 rcall	_Get_Character

_PutCharacter_End:
 pop	TempH								;increment current digit address
 inc	TempH
 pop	XH
 pop	XL
 pop	ZH
 pop	ZL
 pop	Di1
 pop	Di0 
 ret

_Get_Character:
 ldi	ZH, high(LCD_Character_Table*2)			;load character bitmap pointer
 ldi	ZL, low(LCD_Character_Table*2)
 ldi	Temp3, 2
 mul	Temp, Temp3
 add	ZL, r0
 adc	ZH, r1									;add character value to pointer
 lpm	Di1, Z+									;store character bitmap
 lpm	Di0, Z
 
 ldi	ZH, high(LCD_Segment_Table*2)			;load character segment bit position pointer
 ldi	ZL, low(LCD_Segment_Table*2)
 ldi	Temp3, 14								
 andi	Temp4, $0F
 mul	Temp4, Temp3
 add	ZL, r0									;add digit address to pointer
 adc	ZH, r1

 ldi	Temp4, 14

_CharacterLoop: 
 bst	Di0, 0									;mask LSB of character bitmap and store it into T-flag
 lsr	Di1										;shift bitmap right 
 ror	Di0
 lpm	TempH, Z+								;load segment position (combined LCDDR-Byte/Bit position)
  
 mov	Temp, TempH
 andi	TempH, 0b11111000						;get byte number 
 lsr	TempH
 lsr	TempH
 lsr	TempH
 push	XL										;save current x-pointer before addition
 push	XH
 add	XL, TempH
 adc	XH, Null 
 ld		Temp3, X 								;load current byte content
 mov	TempH, Temp
 andi	TempH, 0b00000111						;mask bit position
 ldi	Temp, 0b00000001
 
_CharLoop1:
 cpi	TempH, 0								;get bit position by shifting 
 breq	_CharLoop2
 lsl	Temp
 dec	TempH
 rjmp	_CharLoop1

_CharLoop2:
 brts	_CharBitSet								;check if setting or clearing bit in memory

_CharBitClear:
 com	Temp
 and 	Temp3, Temp
 st		X, Temp3
 rjmp	_CharLoop3

_CharBitSet:
 or 	Temp3, Temp
 st		X, Temp3
 
_CharLoop3:
 pop	XH										;restore current x-pointer
 pop	XL 
 dec	Temp4
 brne	_CharacterLoop
 ret



;----------------------------------------------
Copy_DisplayBuffers:
 ldi	TempH, 20
 lds	Temp, DispTimer
 sbrc	Temp, 1
 rjmp	_C_DB2

 ldi	XL, low(DisplayBuffer1)
 ldi	XH, high(DisplayBuffer1)
 rjmp	_C_DB 

_C_DB2:
 ldi	XL, low(DisplayBuffer2)
 ldi	XH, high(DisplayBuffer2)
  
_C_DB:
 ldi	YL, low (LCDDR0)
 ldi	YH, high (LCDDR0)

_C_DB1:
 ld		Temp, X+
 st		Y+, Temp
 dec	TempH
 brne	_C_DB1 
 ret


;------------------------------------------------------------------------
; write bargraph to LCD
; Input:	Temp: Bargraph position to write, MSB 0: clear bit, MSB1: set bit 
;			TempH: high nibble: Buffer number 1..3 (3 means both display buffers)
; Output:	none
; Used:		Temp3, ZL, ZH, XL, XH

;------------------------------------------------------------------------
PutBargraph:
 push	ZL
 push	ZH
 ldi	ZL, low(Bargraph_Table*2)
 ldi	ZH, high(Bargraph_Table*2)				;load character segment bit position pointer
 rjmp	_OutSymbols

;------------------------------------------------------------------------
; write Day of Week to LCD
; Input:	Temp: Day of Week position to set
;			TempH: high nibble: Buffer number 1..3 (3 means both display buffers)
; Output:	none
; Used:		Temp3, ZL, ZH, XL, XH

;------------------------------------------------------------------------
PutWeekDay:
 push	ZL
 push	ZH
 ldi	ZH, high(WDayTable*2)						;load character segment bit position pointer
 ldi	ZL, low(WDayTable*2)
 rjmp	_OutSymbols




;------------------------------------------------------------------------
; write Symbol to LCD
; Input:	Temp: Symbol number, MSB 1: set symbol // MSB 0: clear symbol
;			TempH: high nibble: Buffer number 1..3 (3 means both display buffers)
; Output:	none
; Used:		Temp3, ZL, ZH, XL, XH


 ;------------------------------------------------------------------------
PutSymbol:
 push	ZL
 push	ZH
 ldi	ZL, low(SymbolTable*2)
 ldi	ZH, high(SymbolTable*2)					;load character segment bit position pointer


_OutSymbols:
 push	XL
 push	XH
 push	TempH
 push	Temp3
 
 andi	TempH, $F0									;mask buffer to write to
 swap	TempH
 cpi	TempH, 1
 breq	_Symbol1
 cpi	TempH, 2
 breq	_Symbol2
 cpi	TempH, 3
 breq	_Symbol3
 rjmp	_PSym_End

_Symbol1:
 ldi	XL, low(DisplayBuffer1)
 ldi	XH, high(DisplayBuffer1)
 rcall	_WriteSymbol
 rjmp	_PSym_End

_Symbol2:
 ldi	XL, low(DisplayBuffer2)
 ldi	XH, high(DisplayBuffer2)
 rcall	_WriteSymbol
 rjmp	_PSym_End

_Symbol3:
 ldi	XL, low(DisplayBuffer1)
 ldi	XH, high(DisplayBuffer1)
 push	Temp
 push	ZL
 push	ZH
 rcall	_WriteSymbol
 pop	ZH										;restore pointer and data for 2nd run
 pop	ZL 
 pop	Temp
 rjmp	_Symbol2
 
_PSym_End:
 pop	Temp3
 pop	TempH
 pop	XH
 pop	XL
 pop	ZH
 pop	ZL
 ret
  
_WriteSymbol:
 bst	Temp, 7									;store MSB in T-Flag
 andi	Temp, 0b00011111
 add	ZL, Temp								;add digit address to pointer
 adc	ZH, Null

 lpm	TempH, Z+								;load segment position (combined LCDDR-Byte/Bit position)
 
 mov	Temp, TempH
 andi	TempH, 0b11111000						;get byte number 
 lsr	TempH
 lsr	TempH
 lsr	TempH
 add	XL, TempH
 adc	XH, Null 
 ld		Temp3, X 								;load current byte content
 mov	TempH, Temp
 andi	TempH, 0b00000111						;mask bit position
 ldi	Temp, 0b00000001
 
_SymbolLoop1:
 cpi	TempH, 0								;get bit position by shifting 
 breq	_SymbolLoop2
 lsl	Temp
 dec	TempH
 rjmp	_SymbolLoop1

_SymbolLoop2:
 brts	_SymbolBitSet							;check if setting or clearing bit in memory

_SymbolBitClear:
 com	Temp
 and 	Temp3, Temp
 st		X, Temp3
 ret

_SymbolBitSet:
 or 	Temp3, Temp
 st		X, Temp3
 ret

;---------------------------------
SetColon:
 lds	Temp, DisplayBuffer1+16				;set colon between segment 1 and 2
 ori	Temp, 0b10000000
 sts	DisplayBuffer1+16, Temp
 lds	Temp, DisplayBuffer2+16				;set colon between segment 1 and 2
 ori	Temp, 0b10000000
 sts	DisplayBuffer2+16, Temp
 ret

;---------------------------------
ClearColon:
 lds	Temp, DisplayBuffer1+16				;clear colon between segment 1 and 2
 andi	Temp, 0b01111111
 sts	DisplayBuffer1+16, Temp
 lds	Temp, DisplayBuffer2+16				
 andi	Temp, 0b01111111
 sts	DisplayBuffer2+16, Temp
 ret

;---------------------------------
SetPoint:
 lds	Temp, DisplayBuffer1+7				;set point between segment 1 and 2
 ori	Temp, 0b10000000
 sts	DisplayBuffer1+7, Temp
 lds	Temp, DisplayBuffer2+7				 
 ori	Temp, 0b10000000
 sts	DisplayBuffer2+7, Temp
 ret

;---------------------------------
ClearPoint:
 lds	Temp, DisplayBuffer1+7
 andi	Temp, 0b01111111					;clear point between segment 1 and 2
 sts	DisplayBuffer1+7, Temp
 lds	Temp, DisplayBuffer2+7
 andi	Temp, 0b01111111				 
 sts	DisplayBuffer2+7, Temp
 ret


 ;------------------------------------------------------------------------
CopyWeekDays1_2:
 lds	Temp, DisplayBuffer1+16
 lds	TempH, DisplayBuffer2+16
 andi	Temp, 0b00111111
 andi	TempH, 0b11000000
 or		TempH, Temp
 sts	DisplayBuffer2+16, TempH
 lds	Temp, DisplayBuffer1+11
 lds	TempH, DisplayBuffer2+11
 andi	Temp, 0b10000000
 andi	TempH, 0b01111111
 or		TempH, Temp
 sts	DisplayBuffer2+11, TempH
 ret


;------------------------------------------------------------------------
ClearWeekDays: 
 lds	Temp3, DisplayBuffer1+16					;clear all days of week on LCD
 andi	Temp3, 0b11000000
 sts	DisplayBuffer1+16, Temp3
 lds	Temp3, DisplayBuffer2+16
 andi	Temp3, 0b11000000
 sts	DisplayBuffer2+16, Temp3
 lds	Temp3, DisplayBuffer1+11					
 andi	Temp3, 0b01111111
 sts	DisplayBuffer1+11, Temp3
 lds	Temp3, DisplayBuffer2+11
 andi	Temp3, 0b01111111
 sts	DisplayBuffer2+11, Temp3
 ret


;------------------------------------------------------------------------
ClearSymbols: 
 lds	Temp, DisplayBuffer1+3		;InHouse
 andi	Temp, 0b11111110
 sts	DisplayBuffer1+3, Temp 
 lds	Temp, DisplayBuffer2+3
 andi	Temp, 0b11111110
 sts	DisplayBuffer2+3, Temp 

 lds	Temp, DisplayBuffer1+8		;OffHouse
 andi	Temp, 0b11111110
 sts	DisplayBuffer1+8, Temp 
 lds	Temp, DisplayBuffer2+8
 andi	Temp, 0b11111110
 sts	DisplayBuffer2+8, Temp 

 lds	Temp, DisplayBuffer1+13		;Moon
 andi	Temp, 0b11111110
 sts	DisplayBuffer1+13, Temp 
 lds	Temp, DisplayBuffer2+13
 andi	Temp, 0b11111110
 sts	DisplayBuffer2+13, Temp 
 ret