;****************************************************
;		Sparmatic-Zero Main Assembly File			*
;****************************************************


.include "m169Pdef.inc"					;MCU description
.include "Def_Equ.inc"					;definition file
.include "Init.inc"						;initialisation file

rcall	StartMain


MainLoop:
 rcall	Adaptation
 
 sbic	Status0, _Adapt
 call	Measure_Motor_Current			;execute in adaptation mode only

 sbis	Status0, _Adapt					;don´t execute in adaptation mode
 rcall	Show_Current_Temperature
  
 sbis	Status0, _Adapt					;don´t execute in adaptation mode
 rcall	Show_Current_Time
 
 sbis	Status0, _Adapt					;don´t execute in adaptation mode
 rcall	User_Action
  
 rcall	Calc_Temperature
 rcall	CheckPosition
 rcall	MotorControl
 call	Clock
 rcall	Copy_DisplayBuffers 

 ;call	FullDebug
 ;call	DebugSniffer 
 

 ;rcall	BargraphTest


 GoSleep:
 cpi	ActiveCT, 0
 brne	_GS1  

 in		Temp, PinB							;read button state before entering sleep
 andi	Temp, 0b10000001					;if both rotary pins are connected to GND, disable pullups to save power (~150µA worst case)
 brne	_GS1
 
 cbi	PortB, 7							
 cbi	PortB, 0
 
_GS1:
 cbi	DebugPort, Debug1
 ldi	Temp, (1<<SM1) | (1<<SM0) | (1<<SE)	;Power save sleep mode
 out	SMCR, Temp

 sleep
 
 sbi	PortB, 7							;re-enable rotary pullups
 sbi	PortB, 0
  
 ldi	Temp, (1<<SM1) | (1<<SM0) | (0<<SE)	;disable sleep
 out	SMCR, Temp
 sbi	DebugPort, Debug1

 rjmp	MainLoop



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




;-------------------------------------------
 StartMain:
 ldi	Temp, 5
 sts	BeepPulseLen, Temp

 ldi	Temp, 200
 sts	BeepLen, Temp

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
 ldi	Temp, 10
 sts	ValveTopL, Temp
 sts	ValveTopH, Null
 ldi	Temp, $A0
 sts	OutLimitL, Temp
 sts	ValveBOTL, Temp
 ldi	Temp, $01
 sts	OutLimitH, Temp
 sts	ValveBOTH, Temp
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
 in 	Temp, Status1
 cbr	Temp, (1<<_NewTemp)
 out 	Status1, Temp

 cbi	Internal_NTC_DDR, Internal_NTC_On
 cbi	Internal_NTC_Port, Internal_NTC_On	;switch NTC voltage divider off
 ldi	Temp, 0b00000100					;ADC disable, ADC Prescaler 16
 sts	ADCSRA, Temp

 lds	Temp, ADCL
 lds	TempH, ADCH
 sts	NTCIntL, Temp						;store NTC voltage
 sts	NTCIntH, TempH
 
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
 sbis	Status0, _Adapt
 rcall	Regulate
 ret


;--------------------------------------------------
; PID-regulation function
Regulate:
  /*lds	Di0, ValveTopL
 lds	Di1, ValveTopH

 in		Temp, Status1
 sbrs	Temp, _PowBad
 rjmp	_Regulate1
 
 ldi	Temp, 40				
 sub 	Di0, Temp							;park position, if bad battery power
 sbc	Di1, Null
 sts	PositionL, Di0
 sts	PositionH, Di1
 ret*/

_Regulate1:
 lds	Temp, TempIntL						;NTC temperature
 lds	TempH, TempIntH

 lds	Di0, SetTempL						;current set temperature
 lds	Di1, SetTempH

 sub	Di0, Temp							;calculate 'e' (feedback offset)
 sbc	Di1, TempH
 movw	TempH:Temp,Di1:Di0		
 brpl	_E_plus								;limit 'e' to positive values

 clr	Temp
 clr	TempH

_E_plus:
 sts	EkL, Temp							;store current feedback offset
 sts	EkH, TempH

 lds	Temp, RegIntL						;load previous integral value
 lds	TempH, RegIntH

 add	Temp, Di0							;add current feedback offset
 adc	TempH, Di1
 brpl	_Regulate2							;limit to positive values

 clr	Temp
 clr	TempH

_Regulate2:
 lds	Temp3, ValveBOTL					;limit integral part positive Value to 1/2 valve way
 lds	Temp4, ValveBOTH
 lsr	Temp4
 ror	Temp3 
 cp 	Temp, Temp3
 cpc	TempH, Temp4
 brcs	_Regulate3
 movw	TempH:Temp, Temp4:Temp3
   
_Regulate3:
 sts	RegIntL, Temp						;store new integral part
 sts	RegIntH, TempH
 
 ldi	Temp5, Reg_I_Mult
 clr	Di2
 fmul	Temp, Temp5							;multiply with fractional I multiplier to 'I' part
 movw	Di1:Di0, r1:r0
 adc	Di2, Null
 fmul  	TempH, Temp5
 adc	Di1, r0
 adc	Di2, r1
 mov	Temp, Di1
 mov	TempH, Di2
 
 lds	Di0, EkL							;load 'e'
 lds	Di1, EkH

 clr	Temp5
 ldi	Temp3, 4							;D threshold
 cp		Di0, Temp3
 cpc	Di1, Null
 brcs	_Regulate4
 ldi	Temp5, Reg_D_Factor					;D-factor

_Regulate4:
 ldi	Temp3, Reg_P_Mult
 
 mul	Di0, Temp3							;multiply 'e' with P multiplier to 'P' part
 movw	Di3:Di2, r1:r0
 mul	Di1, Temp3
 add	Di3, r0

 add	Temp, Di2							;add P part to I part
 adc	TempH, Di3

 add	Temp, Temp5							;add D-part
 adc	TempH, Null

 sts	RegValL, Temp
 sts	RegValH, TempH


_SetPosition:
 lds	Temp3, ValveBOTL					;Load fully closed valve value
 lds	Temp4, ValveBOTH
 
 sub	Temp3, Temp							;subtract regulation value
 sbc	Temp4, TempH
 brmi	_SP0								;avoid negative result
 
 lds	Temp, ValveTOPL
 lds	TempH, ValveTOPH
 cp 	Temp3, Temp							;limit to valve open position
 cpc	Temp4, TempH
 brcc	_SP1

_SP0: 
 lds	Temp3, ValveTOPL
 lds	Temp4, ValveTopH

_SP1:
 sbic	Status0, _MotRun					;do not store new position values while motor is running
 ret

 sts	PositionL, Temp3
 sts	PositionH, Temp4
 sbi	Status0, _MotOn 
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



/*


;-------------------------------------------------------
CopyEEPROMToRAM:
 ldi	XL, low (Prog1AHour)
 ldi	XH, high (Prog1AHour)
 ldi	YL, low (TimerCopy)
 ldi	YH, high (TimerCopy)
 ldi	Temp4, CopyTop


_CEETRAMLoop:
 mov	Temp3, YH
 mov	TempH, YL
 rcall	EERead
 st		X+, Temp
 adiw	YH:YL, 1
 dec	Temp4
 brne	_CEETRAMLoop
 ret

;-------------------------------------------------------
CopyRAMToEEPROM:
 ldi	XL, low (Prog1AHour)
 ldi	XH, high (Prog1AHour)
 ldi	YL, low (TimerCopy)
 ldi	YH, high (TimerCopy)
 ldi	Temp4, CopyTop


_CRAMTEELoop:
 ld		Temp, X+
 mov	Di0, Temp

 mov	Temp3, YH
 mov	TempH, YL
 rcall	EERead
 cp		Di0, Temp
 breq	_CRAMTEE1
 
 mov	Temp, Di0
 mov	Temp3, YH
 mov	TempH, YL
 rcall	EEWrite


_CRAMTEE1:
 adiw	YH:YL, 1
 dec	Temp4
 brne	_CRAMTEELoop
; cbi	Status0, _NewEE
 ret
*/

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




.include	"LCD_Driver.inc"		;Display handler
.include	"LCDTable.inc"
.include	"Debug.inc"
.include	"Motor.inc"
.include	"User.inc"
.include	"ISR.inc"
.include	"ADC.inc"
.include	"RTC.inc"

;.include	"RFM12_S868.inc"
;.include	"Sniffer.inc"




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
.db $00, $00				;MODE   ID0
	.db	$00, $10			;MANU   ID1 
		.db	$00, $11		;MEnt	ID2
	.db $00, $20			;AUTO   ID3
		.db $00, $21		;AEnt	ID4
.db $01, $00				;PROG	ID5
	.db	$01, $10 			;TAG1	ID6
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
.db $02, $00				;TEMP	ID64 
	.db	$02, $10			;onT 	ID65
	.db	$02, $20			;offT	ID66
.db $03, $00				;ZEIT	ID67
	.db	$03, $10			;set year		ID68
	.db	$03, $20			;set month		ID69
	.db	$03, $30			;set date		ID70
	.db	$03, $40			;set hour		ID71
	.db	$03, $50			;set minute		ID72
.db $04, $00				;FENS			ID73
	.db	$04, $10			;HOCH			ID74
		.db $04, $11		;time resume	ID75
	.db	$04, $20			;MITT			ID76
		.db $04, $21		;time resume	ID77
	.db	$04, $30			;NIED			ID78
		.db $04, $31		;time resume	ID79
.db  $05, $00				;RESET			ID80
	.db	 $05, $10			;OK				ID81
.db $06, $00				;ADAP			ID82	
	.db	$06, $10			;ADAP			ID83
.db $07, $00				;URLA			ID84
	.db	$07, $10			;set date		ID85
	.db	$07, $20			;set hour		ID86
	.db	$07, $30			;set temp		ID87
.db $08, $00				;INST			ID88
	.db	$08, $10			;<<<<			ID89
.db $09, $00				;OFFS			ID90
	.db	$09, $10			;set temp		ID91
.db $0A, $00				;DBUG			ID92
	.db	$0A, $10			;Firmware		ID93
	.db	$0A, $20			;ReflexCounter	ID94
	.db	$0A, $30			;Reg Value		ID95




.db $FF, $FF				;menu end