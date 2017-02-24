
.cseg
;Interruptvektoren 
.org    $0000	
 jmp	RESET				;Reset
 jmp	RESET				;Int0
 jmp	PinChange0			;PCInt0
 jmp	PinChange1			;PCInt1
 jmp	RESET				;T2Comp
 jmp	SystemTime			;T2Ov
 jmp	RESET				;T1ICP
 jmp	RESET				;T1CompA 
 jmp	RESET				;T1CompB 
 jmp	RESET				;T1Ov
 jmp	T0_CMP				;T0Comp
 jmp	RESET				;T0Ov
 jmp	RESET				;SPI_STC
 jmp	RESET				;USART_RXC
 jmp	RESET				;USART_DRE
 jmp	RESET				;USART_TXC
 jmp	RESET				;USI_Start
 jmp	RESET				;USI_Ovw 
 jmp	RESET				;ANA_Comp
 jmp	RESET				;ADC_CC
 jmp	RESET				;EE_RDY
 jmp	RESET				;SPM_RDY
 jmp	LCD_FrameInt		;LCD_Start of Frame



 
RESET:
 cli								;Interrupts global ausschalten
 

;initialise Stackpointer
 ldi 	Temp, high(RAMEND)		
 out 	SPH, Temp
 ldi 	Temp, low(RAMEND)
 out 	SPL, Temp

 clr	Null


;Initialising Ports

;		  (-)					Sniffer	   Debug1	  Debug2	 (+)
;RotaryA  "OK"  "MENU"  "TIME"  MISO/USB2  MOSI/USB3  SCK/USB4  RotaryB
 ldi	Temp,  0b00000010
 ldi	TempH, 0b11111101
 out	DDRB, Temp
 out	PortB, TempH



;MotorA  MotorB  Backlight  SCL_H2  AIN1_H2  REFLX_on  RFLX_in  1M_0V
 ldi	Temp,  0b00100100
 ldi	TempH, 0b00111000
 out	DDRE, Temp
 out	PortE, TempH


;JTAG TDI  JTAG TDO  JTAG TMS  JTAG TCK  Int_NTC_on  MOT_Cur  Int_NTC  RFLX_ADC

 ldi	Temp,  0b00000000
 ldi	TempH, 0b11110000
 out	DDRF, Temp
 out	PortF, TempH

 ldi	Temp, 0b00001111
 sts	DIDR0, Temp

 ldi	Temp, (1<<PRTIM1) | (1<<PRSPI) | (1<<PRUSART0)					;disable Timer1, SPI and UART
 sts	PRR, Temp

 ;clear SRAM variables 
 ldi	XH, $01
 ldi	XL, $00
 
_CLSR:
 st		X+, Null
 cpi	XH, $04
 brne	_CLSR


 

;setup LCD-Controller
 ldi	Temp, (1<<LCDCS) | (3<<LCDMUX0) | (7<<LCDPM0)
 sts	LCDCRB, Temp
 ldi	Temp, (0<<LCDPS0) | (3<<LCDCD0)		;64Hz @ 32768Khz
 sts	LCDFRR, Temp
 ldi	Temp, (1<<LCDDC2) | (1<<LCDDC1) | (1<<LCDDC0) | (1<<LCDCC3) | (1<<LCDCC2) | (0<<LCDCC1) | (1<<LCDCC1)		;Contrast setting: drivers 50% on, 3.25Vlcd
 sts	LCDCCR, Temp
 ldi	Temp, (1<<LCDEN) | (1<<LCDIE) | (1<<LCDAB)		;enable LCD controller, LCD-Frame Interrupt, low power waveform
 sts	LCDCRA, Temp
 
 ldi	ActiveCT, 3



;setup ADC
 ldi	Temp, 0b01000001				;AVcc as reference, channel2, iNTCon
 sts	ADMUX, Temp
 ldi	Temp, 0b00010011				;Prescaler 8, 62.5kHz
 sts	ADCSRA, Temp
 ldi	Temp, 0b00000111
 sts	DIDR0, Temp


;setup PinChange for Buttons:
 ldi	Temp, 0b11110001
 sts	PCMSK1, Temp

;setup PinChange for Reflex coupler / Battery remove alert:
 ldi	Temp, 0b00000011
 sts	PCMSK0, Temp
 
 ldi	Temp, (1<<PCIE1) | (1<<PCIE0)			;enable pin change on PCINT15...8 // PCINT7...0
 out	EIMSK, Temp


 ldi	Temp, 0b00001001						;CTC, prescaler 1
 out	TCCR0A, Temp
 ldi	Temp, 3
 out	OCR0A, Temp




;setup Timer2 for SystemTime
 ldi	Temp, (0<<CS22)| (1<<CS21) | (1<<CS20)	;Normal mode, prescaler 32
 sts	TCCR2A, Temp
 ldi	Temp, (1<<TOIE2)						;Overflow interrupt T2_A enabled
 sts	TIMSK2, Temp 
 ldi	Temp, (1<<AS2)
 sts	ASSR, Temp
 







/* 

;initialising variables
 out	Status0, Null
 out	Status1, Null

 clr	Prescaler1
 ldi	Temp, 4
 sts	PSC1, Temp				;init prescaler1 interval (default 4)
 ldi	Temp, 35
 sts	TimeAdjust, Temp

 ldi	Temp, 10
 sts	Years, Temp

 ldi	TempH, low(TimerCopy)
 ldi	Temp3, high(TimerCopy)
 rcall	EERead
 cpi	Temp, $FF				;if first EEPROM cell is not empty, reload EEPROM into RAM
 breq	_SetupDefaultTimer
 rcall	CopyEEPROMToRAM
 rjmp	_Init_End


_SetupDefaultTimer:
 ldi	Temp, 5					;Monday on
 ldi	TempH, 30
 sts	Prog1AHour, Temp
 sts	Prog1AMin, TempH
 ldi	Temp, 6					;Tuesday...Friday on
 ldi	TempH, 30
 sts	Prog2AHour, Temp
 sts	Prog2AMin, TempH
 sts	Prog3AHour, Temp
 sts	Prog3AMin, TempH
 sts	Prog4AHour, Temp
 sts	Prog4AMin, TempH
 sts	Prog5AHour, Temp
 sts	Prog5AMin, TempH
 ldi	Temp, $AA				;Saturday...Sunday on
 ldi	TempH, $AA
 sts	Prog6AHour, Temp
 sts	Prog6AMin, TempH
 sts	Prog7AHour, Temp
 sts	Prog7AMin, TempH

 ldi	Temp, 18				;Monday...Friday off
 ldi	TempH, 00
 sts	Prog1BHour, Temp
 sts	Prog1BMin, TempH
 sts	Prog2BHour, Temp
 sts	Prog2BMin, TempH
 sts	Prog3BHour, Temp
 sts	Prog3BMin, TempH
 sts	Prog4BHour, Temp
 sts	Prog4BMin, TempH
 sts	Prog5BHour, Temp
 sts	Prog5BMin, TempH
 ldi	Temp, $AA				;Saturday...Sunday off
 ldi	TempH, $AA
 sts	Prog6BHour, Temp
 sts	Prog6BMin, TempH
 sts	Prog7BHour, Temp
 sts	Prog7BMin, TempH

 ldi	Temp, $AA				;Monday...Friday 2nd on
 ldi	TempH, $AA
 sts	Prog1CHour, Temp
 sts	Prog1CMin, TempH
 sts	Prog2CHour, Temp
 sts	Prog2CMin, TempH
 sts	Prog3CHour, Temp
 sts	Prog3CMin, TempH
 sts	Prog4CHour, Temp
 sts	Prog4CMin, TempH
 sts	Prog5CHour, Temp
 sts	Prog5CMin, TempH
 ldi	Temp, $AA				;Saturday...Sunday 2nd on
 ldi	TempH, $AA
 sts	Prog6CHour, Temp
 sts	Prog6CMin, TempH
 sts	Prog7CHour, Temp
 sts	Prog7CMin, TempH

 ldi	Temp, $AA				;Monday...Friday 2nd off
 ldi	TempH, $AA
 sts	Prog1DHour, Temp
 sts	Prog1DMin, TempH
 sts	Prog2DHour, Temp
 sts	Prog2DMin, TempH
 sts	Prog3DHour, Temp
 sts	Prog3DMin, TempH
 sts	Prog4DHour, Temp
 sts	Prog4DMin, TempH
 sts	Prog5DHour, Temp
 sts	Prog5DMin, TempH
 ldi	Temp, $AA				;Saturday...Sunday 2nd off
 ldi	TempH, $AA
 sts	Prog6DHour, Temp
 sts	Prog6DMin, TempH
 sts	Prog7DHour, Temp
 sts	Prog7DMin, TempH

 ldi	Temp, low(225)			;default value for SunTemp
 ldi	TempH, high(225)
 sts	SunTempL, Temp
 sts	SunTempH, TempH
 ldi	Temp, low(180)			;default value for MoonTemp
 ldi	TempH, high(180)
 sts	MoonTempL, Temp
 sts	MoonTempH, TempH

 ldi	Temp, $08					
 sts	RFMTune, Temp


_Init_End:
 ldi	TempH, $FE				;RFM_Software-reset
 ldi	Temp, $00		
 call	Read_WriteSPI


RFM_Start:
 wdr
 call	WaitRFM_Reset
 call	Get_RFM_Status
 cp		RFM12_ReadH, Null
 breq	RFM_Start
 ser	Temp
 cp		RFM12_ReadH, Temp
 breq	NoRFM

 lds	Temp, LCDDR12
 sbr	Temp, (1<<RadioTower)		;set radio tower symbol, if RFM is present
 sts	LCDDR12, Temp
 call	RFM12_Init
 call	RFM12_AllOff 
 in		Temp, Status2
 sbr	Temp, (1<<_RFMPresent)
 out	Status2, Temp


NoRFM:
 ldi	Temp, $AA
 sts	RFM_TX_Buffer, Temp
 ldi	Temp, $2D
 sts	RFM_TX_Buffer+1, Temp
 ldi	Temp, $D4
 sts	RFM_TX_Buffer+2, Temp
 
*/

 clr	Prescaler1
 ldi	Temp, 4
 sts	PSC1, Temp					;init prescaler1 interval (default 4)
 ldi	Temp, 35
 sts	TimeAdjust, Temp
 
 ldi	Temp, $FF
 sts	ADCPrescaler, Temp

 /*ldi	Temp, 10
 sts	ValveTopL, Temp
 sts	ValveTopH, Null*/

 ldi	Temp, 200					;init Set Temperature to 20.0°C
 sts	SetTempL, Temp
 sts	SetTempH, Null

 cbi	BacklightPort, Backlight_On
 ;cbi	DebugPort, Debug1

 rcall	ReadBack_Time


 sei								;global interrupt enable

;end of file
