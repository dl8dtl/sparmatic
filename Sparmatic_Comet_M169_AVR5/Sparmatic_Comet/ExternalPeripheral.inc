

;-----------------------------------------------------------------------------
;Soft SPI, uses MOSI, MISO and SCK // 1 Data transmit on one call
;Inputs:	Temp: Transmit data
;Output:	Temp: Received data
;Used:		Temp, TempH

Soft_SPI:
 ldi	TempH, 8
 lsl	Temp

_SSPI_Loop:
 brcs	_SSPI_H_Out

_SSPI_L_Out:
 cbi	SSPI_PORT, SSPI_MOSI
 rjmp	_SSPI_CLOCK

_SSPI_H_Out:
 sbi	SSPI_PORT, SSPI_MOSI
 nop
 nop

_SSPI_CLOCK:
 sbi	SSPI_PORT, SSPI_SCK
 nop
 nop
 clc	
 sbic	SSPI_PIN, SSPI_MISO
 sec
 rol	Temp
 cbi	SSPI_PORT, SSPI_SCK
 dec	TempH
 brne	_SSPI_LOOP
 cbi	SSPI_PORT, SSPI_MOSI
 ret