/*
 * Sparmatic/Comet (V2) Electronic Heating Valve Operating Firmware
 *
 * Translation of Knut Ballhause's Assembly Program into C,
 * translation by Joerg Wunsch.
 *
 * Placed into the Public Domain.
 */

/* $Id: defines.h,v c9410094fdb0 2017/03/24 12:39:06 "Joerg $ */

#include <stdint.h>

#define Reg_P_Mult		12				/* regulator P multiplier */
#define Reg_I_Mult		0b00110000		/* regulator I multiplier, fractional number (0b10000000=1, 0b01000000=0.5, 0b00100000=0.25 ) */
#define Reg_D_Factor	70

#define MaxAdaptWay		400


/* internal NTC sensor */
#define Internal_NTC_Port PORTF
#define Internal_NTC_DDR DDRF
#define Internal_NTC_On	_BV(3)

/* LCD backlight (optional) */
#define BacklightPort	PORTE
#define Backlight_On	_BV(5)

/* Reflex optocoupler */
#define LightSensPort	PORTE
#define LightSensPin	PINE
#define LightSens_LED	_BV(2)
#define LightSens_Trans	_BV(1)

/* Motor full bridge */
#define MotorPort		PORTE
#define MotorDDR		DDRE
#define Motor_Close		_BV(6)
#define Motor_Open		_BV(7)

/* Soft_SPI */
#define SSPI_PORT		PORTB
#define SSPI_DDR		DDRB
#define SSPI_PIN		PINB
#define SSPI_MOSI		_BV(2)
#define SSPI_MISO		_BV(3)
#define SSPI_SCK		_BV(1)

/* Buttons: */
#define OK_Button		4
#define Menu_Button	1
#define Time_Button	2

/* Status0 Flagregister */
#define Status0	GPIOR0
#define MotOn	_BV(0)  // MotorControl() shall take action
#define MotDir	_BV(1)  // Close valve
#define MotRun	_BV(2)  // Motor hardware is running
#define Adapt	_BV(3)
#define NewButton	_BV(4)
#define MenuUpDn	_BV(5)
#define MenuOn	_BV(6)
#define MenuWork	_BV(7)

/* Status1 */
#define Status1	GPIOR1
#define PowLow	_BV(0)
#define PowBad	_BV(1)
#define Descale	_BV(2)
#define DST_OnOff	_BV(3)
#define NewTemp	_BV(4)
#define DebugMode	_BV(5)
#define ToggleFlag	_BV(6)
#define SecondTick	_BV(7)

/* Status2 */
#define Status2	GPIOR2
#define TX_SSPI _BV(0)




/* MotTimeOut status bits */
#define BotLimit		_BV(6)
#define TopLimit		_BV(7)


#define TStatic			0x00
#define TRise			0x01
#define TFall			0x02

#define FW_Version		003


/* Symbol Definition */
#define LCD_HourBar_CLR	0x00
#define LCD_HourBar_SET	0x80
#define LCD_Auto_CLR	0x01
#define LCD_Auto_SET	0x81
#define LCD_Manu_CLR	0x02
#define LCD_Manu_SET	0x82
#define LCD_Case_CLR	0x03
#define LCD_Case_SET	0x83
#define LCD_Tower_CLR	0x04
#define LCD_Tower_SET	0x84
#define LCD_InHouse_CLR	0x05
#define LCD_InHouse_SET	0x85
#define LCD_OffHouse_CLR 0x06
#define LCD_OffHouse_SET 0x86
#define LCD_Moon_CLR	0x07
#define LCD_Moon_SET	0x87
#define LCD_Star_CLR	0x08
#define LCD_Star_SET	0x88
#define LCD_Battery_CLR	0x09
#define LCD_Battery_SET	0x89
#define LCD_Lock_CLR	0x0A
#define LCD_Lock_SET	0x8A

#define LCD_Mo_CLR		0x00
#define LCD_Mo_SET		0x80
#define LCD_Di_CLR		0x01
#define LCD_Di_SET		0x81
#define LCD_Mi_CLR		0x02
#define LCD_Mi_SET		0x82
#define LCD_Do_CLR		0x03
#define LCD_Do_SET		0x83
#define LCD_Fr_CLR		0x04
#define LCD_Fr_SET		0x84
#define LCD_Sa_CLR		0x05
#define LCD_Sa_SET		0x85
#define LCD_So_CLR		0x06
#define LCD_So_SET		0x86

// register variables in Knut's ASM code
extern uint8_t MotPWMCT;        /* r4 */
extern uint8_t Prescaler1;      /* r7 */
extern uint8_t ButtonDebounce;     /* r19 */
extern uint8_t MenuID;             /* r0 */
extern uint8_t ActiveCT;           /* r22 */


extern int16_t TempInt;
extern int16_t TempIntOld;
extern int8_t DeltaTemp1;
extern int8_t DeltaTemp2;
extern uint16_t NTCInt;
extern uint8_t TimeAdjust;
extern uint8_t Buttons;
extern uint8_t OldButtons;
extern uint8_t RotaryTemp;
extern uint8_t Rotary;
extern uint16_t RFLXCT;
extern uint16_t Position;
extern uint16_t ValveTop;
//extern uint16_t ValveBot;
extern uint8_t MotTimeOut;
extern uint8_t AdaptStep;
extern uint8_t MotorCurrent;
extern uint8_t FreeMotorCurrent;
extern uint8_t ADCPrescaler;
struct time
{
    uint8_t Seconds;
    uint8_t Minutes;
    uint8_t Hours;
    uint8_t Days;
    uint8_t Months;
    uint8_t Years;
    uint8_t WDays;
};
extern struct time TOD, Urlaub;
extern uint8_t UserDisplay;
extern uint16_t iNTCV;
extern uint8_t PSC1;
extern uint16_t SetTemp;
extern int16_t TempOffset;
extern uint16_t Ek;
extern int8_t RegWay;
extern uint8_t DisplayCT;
extern uint8_t BeepCT;
extern uint8_t BeepLen;
extern uint8_t BeepPulseCT;
extern uint8_t BeepPulseLen;
extern uint8_t MenuLow;
extern uint8_t MenuHigh;
extern uint8_t DispTimer;
extern uint8_t OldBarBlink;
extern uint8_t *BarBase;
extern uint8_t FuzzyVal;
//extern uint8_t FuzzyCounter;
extern uint8_t test;


extern uint16_t InHouseTemp;
extern uint16_t OffHouseTemp;
extern uint16_t NightTemp;
extern uint16_t WindowOpenTemp;

extern uint8_t DisplayBuffer1[20];
extern uint8_t DisplayBuffer2[20];

/* 9 timers for each day (Hot,Cold,Hot,Cold,Hot,Cold,Hot,Cold,Night),
 * 7 days, 1-5 / 1-6 / 1-7 / 6-7 */
#define NTIMERS 11
#define TIMPERDAY 9
extern uint8_t DailyTimer[TIMPERDAY * NTIMERS];


enum fuzzy
{
   FuzzOK = 0,
   FuzzWarm = 0x04,
   FuzzHot = 0x08,
   FuzzCool = 0x0c,
   FuzzCold = 0x10,
   FuzzAbove = 0x14,
   FuzzBelow = 0x18,
};

/* User Interface Modes (0..255) */
enum user_if_modes
{
    NormalMode,
    ProgSunMode,
    ProgMoonMode,

    Prog1AMode,
    Prog1BMode,
    Prog1CMode,
    Prog1DMode,

    Prog2AMode,
    Prog2BMode,
    Prog2CMode,
    Prog2DMode,

    Prog3AMode,
    Prog3BMode,
    Prog3CMode,
    Prog3DMode,

    Prog4AMode,
    Prog4BMode,
    Prog4CMode,
    Prog4DMode,

    Prog5AMode,
    Prog5BMode,
    Prog5CMode,
    Prog5DMode,

    Prog6AMode,
    Prog6BMode,
    Prog6CMode,
    Prog6DMode,

    Prog7AMode,
    Prog7BMode,
    Prog7CMode,
    Prog7DMode,

    ProgYearMode,
    ProgMonthMode,
    ProgDayMode,
    ProgHourMode,
    ProgMinuteMode,

    ProgRFMTuneMode,

    MenuEnd,
};


struct eedata
{
    uint8_t reserved[16]; // avoid using address 0
    struct
    {
        uint8_t Minutes, Hours, Days, Months, Years;
    }
    tod;
    struct
    {
        uint16_t position, valvetop;
    }
    valvestate;
    uint8_t dailytimer[TIMPERDAY * NTIMERS];
    struct
    {
        uint16_t inhouse, offhouse, night, windowopen;
        int16_t offset;
    }
    temperatures;
};
