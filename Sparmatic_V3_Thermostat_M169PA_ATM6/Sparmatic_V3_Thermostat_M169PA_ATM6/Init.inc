
.cseg
;Interruptvektoren 
.org    $0000	
 jmp	RESET				;Reset					0000		
 jmp	RESET				;Int0					0002		
 jmp	PinChange0			;PCInt0					0004		
 jmp	PinChange1			;PCInt1					0006		
 jmp	RESET				;T2Comp					0008
 jmp	SystemTime			;T2Ov					000A
 jmp	RESET				;T1ICP					000C
 jmp	RESET				;T1CompA				000E
 jmp	RESET				;T1CompB				0010
 jmp	RESET				;T1Ov					0012
 jmp	T0_CMP				;T0Comp					0014
 jmp	RESET				;T0Ov					0016
 jmp	RESET				;SPI_STC				0018		
 jmp	RESET				;USART_RXC				001A
 jmp	RESET				;USART_DRE				001C
 jmp	RESET				;USART_TXC				001E
 jmp	RESET				;USI_Start				0020
 jmp	RESET				;USI_Ovw				0022
 jmp	RESET				;ANA_Comp				0024
 jmp	RESET				;ADC_CC					0026
 jmp	RESET				;EE_RDY					0028
 jmp	RESET				;SPM_RDY				002A	
 jmp	LCD_FrameInt		;LCD_Start of Frame		002C

.db 0xF3, 0x18, 0x4B, 0xC9

 
RESET:
 cli								;Interrupts global ausschalten
 

;initialise Stackpointer
 ldi 	Temp, high(RAMEND)		
 out 	SPH, Temp
 ldi 	Temp, low(RAMEND)
 out 	SPL, Temp

 clr	Null


;Initialising Ports

;(-)							Sniffer	   Debug1	  Debug2	 (+)
;RotaryA  "OK"  "MENU"  "TIME"  MISO/USB2  MOSI/USB3  SCK/USB4  RotaryB
 ldi	Temp,  0b00000010
 ldi	TempH, 0b11111101
 out	DDRB, Temp
 out	PortB, TempH

 
;MotorA  MotorB  1M_0V  ----  RFLX_in  REFLX_on  ----  ----
 ldi	Temp,  0b00000100
 ldi	TempH, 0b00010011
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


;setup PinChange for Buttons and Rotary encoder:
 ldi	Temp, 0b11110001
 sts	PCMSK1, Temp

;setup PinChange for Reflex coupler 
 ldi	Temp, 0b00001000
 sts	PCMSK0, Temp
 
 ldi	Temp, (1<<PCIE1) | (1<<PCIE0)			;enable pin change on PCINT15...8 // PCINT7...0
 out	EIMSK, Temp


;setup timer0 for sound
 ldi	Temp, 0b00001001						;CTC, prescaler 1
 out	TCCR0A, Temp
 ldi	Temp, 0
 out	OCR0A, Temp

 
;setup Timer2 for SystemTime
 ldi	Temp, (0<<CS22)| (1<<CS21) | (1<<CS20)	;Normal mode, prescaler 32
 sts	TCCR2A, Temp
 ldi	Temp, (1<<TOIE2)						;Overflow interrupt T2_A enabled
 sts	TIMSK2, Temp 
 ldi	Temp, (1<<AS2)
 sts	ASSR, Temp
 
;initialising variables
 clr	Prescaler1
 ldi	Temp, 4
 sts	PSC1, Temp								;init prescaler1 interval (default 4)
 
 /*ldi	Temp, 40
 sts	ADCPrescaler, Temp*/

 ser	Temp									;disable IO_interface (write 0xFF to BitCT)
 sts	IO_BitCT, Temp


 ldi	Temp, 200								;init Set Temperature to 20.0°C
 sts	SetTempL, Temp
 sts	SetTempH, Null

 cbi	BacklightPort, Backlight_On

 rcall	ReadEEProm2Timers
 rcall	ReadBack_Time
 rcall	ReadEEProm2Temperatures


 lds	Temp, UserSettings1						
 cbr	Temp, (1<<_ExternalTempSensor)			;"sbr" activates external sensor acqusition
 sts	UserSettings1, Temp
 
 sei											;global interrupt enable

//End of file//
