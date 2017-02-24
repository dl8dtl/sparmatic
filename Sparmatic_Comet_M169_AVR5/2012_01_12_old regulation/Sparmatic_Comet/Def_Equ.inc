.def	Null			=r2
.def	SREG_Save		=r3
.def	MotPWMCT		=r4
;.def	RFM12_ReadH		=r5
;.def	RFM12_ReadL		=r6
.def	Prescaler1		=r7
.def	Ch0				=r8
.def	Ch1				=r9
.def	Ch2				=r10

.def	Di0				=r12
.def	Di1				=r13
.def	Di2				=r14
.def	Di3				=r15

.def	Temp			=r16
.def	TempH			=r17
.def	Temp5			=r18
.def	ButtonDebounce	=r19
.def	MenuID			=r20

.def	ActiveCT		=r22

.def	Temp3			=r24
.def	Temp4			=r25


;Constants:
.equ	Reg_P_Mult			=12				;regulator P multiplier
.equ	Reg_I_Mult			=0b00110000		;regulator I multiplier, fractional number (0b10000000=1, 0b01000000=0.5, 0b00100000=0.25 )
.equ	Reg_D_Factor		=70



;internal NTC sensor
.equ	Internal_NTC_Port	=PortF
.equ	Internal_NTC_DDR	=DDRF
.equ	Internal_NTC_On		=3

;LCD backlight (optional)
.equ	BacklightPort		=PortE
.equ	Backlight_On		=5

;Reflex optocoupler
.equ	LightSensPort		=PortE
.equ	LightSensPin		=PinE
.equ	LightSens_LED		=2
.equ	LightSens_Trans		=1

;Motor full bridge
.equ	MotorPort			=PortE
.equ	MotorDDR			=DDRE
.equ	Motor_Close			=6
.equ	Motor_Open			=7

;Debug interface
.equ	DebugPort			=PortB
.equ	DebugPin			=PinB
.equ	SnifPin				=1
.equ	Debug1				=2
.equ	Debug2				=3

;Buttons:
.equ	_OK_Button			=4
.equ	_Menu_Button		=1
.equ	_Time_Button		=2


/*.equ	ButtonIn 	=PinB
.equ	RFM12_Pin	=PinF
.equ	RFM12_Port	=PortF
.equ	RFM12_DI	=4
.equ	RFM12_SCK	=5
.equ	RFM12_nSEL	=6
.equ	RFM12_DO	=7

.equ	RadioTower	=3*/



;Status0 Flagregister
.equ	Status0		=GPIOR0
.equ	_MotOn		=0
.equ	_MotDir		=1
.equ	_MotRun		=2
.equ	_Adapt		=3
.equ	_NewButton	=4
.equ	_MenuUpDn	=5
.equ	_MenuOn		=6
.equ	_MenuWork	=7

;Status1
.equ	Status1		=GPIOR1
.equ	_PowLow		=0
.equ	_PowBad		=1
.equ	_Descale	=2
.equ	_DST_OnOff	=3
.equ	_NewTemp	=4
.equ	_DebugMode	=5
.equ	_ToggleFlag	=6
.equ	_SecondTick	=7

/*;Status2
.equ	Status2		=GPIOR2
.equ	_RFMPresent	=0
.equ	_TX_RFM		=1
.equ	_RX_RFM		=2
.equ	_External	=3
.equ	_NewRXString=4*/


.equ	_PosEqual			=6
.equ	_PosReached			=7



.equ	FW_Version	= 001


;Symbol Definition
.equ	LCD_HourBar_CLR		=$00
.equ	LCD_HourBar_SET		=$80
.equ	LCD_Auto_CLR		=$01
.equ	LCD_Auto_SET		=$81
.equ	LCD_Manu_CLR		=$02
.equ	LCD_Manu_SET		=$82
.equ	LCD_Case_CLR		=$03
.equ	LCD_Case_SET		=$83
.equ	LCD_Tower_CLR		=$04
.equ	LCD_Tower_SET		=$84
.equ	LCD_InHouse_CLR		=$05
.equ	LCD_InHouse_SET		=$85
.equ	LCD_OffHouse_CLR	=$06
.equ	LCD_OffHouse_SET	=$86
.equ	LCD_Moon_CLR		=$07
.equ	LCD_Moon_SET		=$87
.equ	LCD_Star_CLR		=$08
.equ	LCD_Star_SET		=$88
.equ	LCD_Battery_CLR		=$09
.equ	LCD_Battery_SET		=$89
.equ	LCD_Lock_CLR		=$0A
.equ	LCD_Lock_SET		=$8A

.equ	LCD_Mo_CLR			=$00
.equ	LCD_Mo_SET			=$80
.equ	LCD_Di_CLR			=$01
.equ	LCD_Di_SET			=$81
.equ	LCD_Mi_CLR			=$02
.equ	LCD_Mi_SET			=$82
.equ	LCD_Do_CLR			=$03
.equ	LCD_Do_SET			=$83
.equ	LCD_Fr_CLR			=$04
.equ	LCD_Fr_SET			=$84
.equ	LCD_Sa_CLR			=$05
.equ	LCD_Sa_SET			=$85
.equ	LCD_So_CLR			=$06
.equ	LCD_So_SET			=$86



.dseg
;SRAM
.org $0100
TempIntL:					.Byte 1
TempIntH:					.Byte 1
NTCIntL:					.Byte 1
NTCIntH:					.Byte 1
TimeAdjust:					.Byte 1
Buttons:					.Byte 1
OldButtons:					.Byte 1
RotaryTemp:					.Byte 1
Rotary:						.Byte 1
RFLXCTL:					.Byte 1
RFLXCTH:					.Byte 1
PositionL:					.Byte 1
PositionH:					.Byte 1
ValveTopL:					.Byte 1
ValveTopH:					.Byte 1
ValveBotL:					.Byte 1
ValveBotH:					.Byte 1
MotTimeOut:					.Byte 1
AdaptStep:					.Byte 1
MotorCurrent:				.Byte 1
FreeMotorCurrent:			.Byte 1
OutLimitL:					.Byte 1
OutLimitH:					.Byte 1
DebugAddress:				.Byte 1
ADCPrescaler:				.Byte 1
Seconds:					.Byte 1
Minutes:					.Byte 1
Hours:						.Byte 1
Days:						.Byte 1
Months:						.Byte 1
Years:						.Byte 1
WDays:						.Byte 1
UserDisplay:				.Byte 1
iNTCVL:						.Byte 1
iNTCVH:						.Byte 1
PSC1:						.Byte 1
SetTempL:					.Byte 1
SetTempH:					.Byte 1
EkL:						.Byte 1
EkH:						.Byte 1
RegValL:					.Byte 1
RegValH:					.Byte 1
RegIntL:					.Byte 1
RegIntH:					.Byte 1
DisplayCT:					.Byte 1
BeepCT:						.Byte 1
BeepLen:					.Byte 1
BeepPulseCT:				.Byte 1
BeepPulseLen:				.Byte 1
MenuLow:					.Byte 1
MenuHigh:					.Byte 1
DispTimer:					.Byte 1
OldBarBlink:				.Byte 1
BarBaseL:					.Byte 1
BarBaseH:					.Byte 1
test:						.Byte 1


InHouseTempL:				.Byte 1
InHouseTempH:				.Byte 1
OffHouseTempL:				.Byte 1
OffHouseTempH:				.Byte 1
NightTempL:					.Byte 1
NightTempH:					.Byte 1
WindowOpenTempL:			.Byte 1
WindowOpenTempH:			.Byte 1



DisplayBuffer1:				.Byte 20
DisplayBuffer2:				.Byte 20

DailyTimer:					.Byte 90			;9 timers for each day (Hot,Cold,Hot,Cold,Hot,Cold,Hot,Cold,Night), 7 days, 1-5 / 1-6 / 1-7


DebugBuffer:				.Byte 256







;EEPROM:
.eseg
.org	$01E0
Minutes_EE:						.Byte 1
Hours_EE:						.Byte 1
Days_EE:						.Byte 1
Months_EE:						.Byte 1
Years_EE:						.Byte 1





/*.equ	TimerCopy			=$0000				;Timer and settings copy block



;User Interface Modes (0..255)
.equ	NormalMode		=0
.equ	ProgSunMode		=1
.equ	ProgMoonMode	=2

.equ	Prog1AMode		=3
.equ	Prog1BMode		=4
.equ	Prog1CMode		=5
.equ	Prog1DMode		=6

.equ	Prog2AMode		=7
.equ	Prog2BMode		=8
.equ	Prog2CMode		=9
.equ	Prog2DMode		=10

.equ	Prog3AMode		=11
.equ	Prog3BMode		=12
.equ	Prog3CMode		=13
.equ	Prog3DMode		=14

.equ	Prog4AMode		=15
.equ	Prog4BMode		=16
.equ	Prog4CMode		=17
.equ	Prog4DMode		=18

.equ	Prog5AMode		=19
.equ	Prog5BMode		=20
.equ	Prog5CMode		=21
.equ	Prog5DMode		=22

.equ	Prog6AMode		=23
.equ	Prog6BMode		=24
.equ	Prog6CMode		=25
.equ	Prog6DMode		=26

.equ	Prog7AMode		=27
.equ	Prog7BMode		=28
.equ	Prog7CMode		=29
.equ	Prog7DMode		=30

.equ	ProgYearMode	=31
.equ	ProgMonthMode	=32
.equ	ProgDayMode		=33
.equ	ProgHourMode	=34
.equ	ProgMinuteMode	=35

.equ	ProgRFMTuneMode	=36

.equ	MenuEnd			=37

.equ	CopyTop			=61			;Bytes to copy from EEPROM to SRAM and vice versa*/

/*

;Displaymodes (0..255)
.equ	DispPosition	=0
.equ	DispSetTemp		=1
.equ	DispTemperatures=2
.equ	DispSunTemp		=3
.equ	DispMoonTemp	=4
.equ	DispSetYear		=5
.equ	DispSetMonth	=6
.equ	DispSetDay		=7
.equ	DispSetHour		=8
.equ	DispSetMinute	=9
.equ	DispSetProg1	=10
.equ	DispProgRFMTune	=11


;KeyFlags (Bit values)
.equ	PlusFlag		=0
.equ	MinusFlag		=1
.equ	ProgFlag		=2
.equ	AutoFlag		=3


.equ	FirmWareVers	=01*/


;end of file
