.def	Null			=r2
.def	SREG_Save		=r3
.def	MotPWMCT		=r4

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
.def	TemperatureMode	=r21
.def	ActiveCT		=r22

.def	Temp3			=r24
.def	Temp4			=r25





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
.equ	LightSens_Trans		=3

;Motor full bridge
.equ	MotorPort			=PortE
.equ	MotorDDR			=DDRE
.equ	Motor_Close			=6
.equ	Motor_Open			=7


;Buttons:
.equ	_OK_Button			=4
.equ	_Menu_Button		=1
.equ	_Time_Button		=2


;external interface
.equ	IO_Port		=PORTB
.equ	IO_DDR		=DDRB
.equ	IO_PIN		=PINB
.equ	IO_SCK		=3
.equ	IO_SDATA	=2


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
.equ	Status1			=GPIOR1
.equ	_IO_Receive		=0
.equ	_AutoManu		=1
.equ	_Descale		=2
.equ	_DST_OnOff		=3
.equ	_NewTemp		=4
.equ	_SecondTick		=7


;Flags only used in Main
;Status2
.equ	Status2			=GPIOR2
.equ	_NewTempMode	=1
.equ	_FastModeScan	=2
.equ	_IO_Running		=3


;UserSettings1_Bits
.equ	_ValveInverse		=0
.equ	_ExternalTempSensor	=1
.equ	_Holiday			=2
.equ	_2ndHoliday			=3


;MotTimeOut status bits
.equ	_BotLimit			=6
.equ	_TopLimit			=7


;Fuzzy Variable:
.equ	FuzzOK				=$00	;00
.equ	FuzzWarm			=$04	;04
.equ	FuzzHot				=$08	;08
.equ	FuzzCool			=$0C	;12
.equ	FuzzCold			=$10	;16
.equ	FuzzAbove			=$14	;20
.equ	FuzzBelow			=$18	;24


.equ	TStatic				=$00
.equ	TRise				=$01
.equ	TFall				=$02

.equ	NoTempMode			=0
.equ	InHouseMode			=1
.equ	OffHouseMode		=2
.equ	NightMode			=3


.equ	MaxAdaptWay			=380

.equ	Block1_5			=7
.equ	Block1_6			=8
.equ	Block1_7			=9
.equ	BlockHoliday1		=10
.equ	BlockHoliday2		=11

.equ	FW_Version			=007


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
TempIntOldL:				.Byte 1
TempIntOldH:				.Byte 1
DeltaTemp1:					.Byte 1
DeltaTemp2:					.Byte 1
NTCIntL:					.Byte 1
NTCIntH:					.Byte 1
IO_BitCT:					.Byte 1
IO_ByteCT:					.Byte 1
IO_Data:					.Byte 1
IO_DataBuffer:				.Byte 1
IO_Timeout:					.Byte 1
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
MotTimeOut:					.Byte 1
AdaptStep:					.Byte 1
MotorCurrent:				.Byte 1
FreeMotorCurrent:			.Byte 1
ValveMotorCurrent:			.Byte 1
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
RegWay:						.Byte 1
DisplayCT:					.Byte 1
BeepCT:						.Byte 1
BeepLen:					.Byte 1
BeepPulseCT:				.Byte 1
MenuLow:					.Byte 1
MenuHigh:					.Byte 1
DispTimer:					.Byte 1
OldBarBlink:				.Byte 1
BarBaseL:					.Byte 1
BarBaseH:					.Byte 1
FuzzyVal:					.Byte 1
BatteryVoltageL:			.Byte 1
BatteryVoltageH:			.Byte 1
TempModeOld:				.Byte 1
Radio_RXTimeslot:			.Byte 1

UserSettings1:				.Byte 1

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

DailyTimer:					.Byte 108			;9 timers for each day (Hot,Cold,Hot,Cold,Hot,Cold,Hot,Cold,Night) // 7 days / 1-5 / 1-6 / 1-7 / Holiday1 / Holiday2


ExternalDataBuffer:			.Byte 32




;EEPROM:
.eseg
.org 0x0000
 EE_DailyTimer:					.Byte 108

.org	0x00E0
 Minutes_EE:					.Byte 1
 Hours_EE:						.Byte 1
 Days_EE:						.Byte 1
 Months_EE:						.Byte 1
 Years_EE:						.Byte 1
 EE_Temperatures:				.Byte 6


.org	0x0100									;parameter block
 



//End of file//