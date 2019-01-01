/*
 * Sparmatic/Comet (V2) Electronic Heating Valve Operating Firmware
 *
 * Translation of Knut Ballhause's Assembly Program into C,
 * translation by Joerg Wunsch.
 *
 * Placed into the Public Domain.
 */

/* $Id: comet.c,v 2415bf4e2043 2017/03/25 09:29:48 "Joerg $ */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

#include <util/delay.h>

#include "defines.h"

#define DEGREE "@"  // used as degree symbol
#define DELTA  ";"  // used as greek Delta

/* Helper macro for flash strings */
#define FSTR(s) ({static const __flash char __c[] = (s); &__c[0];})

/* These variables use hardwired registers in assembly version */
uint8_t MotPWMCT;        /* r4 */
uint8_t Prescaler1;      /* r7 */
uint8_t ButtonDebounce;     /* r19 */
const __flash struct menuentry *menuID;
uint8_t ActiveCT;           /* r22 */

/* Normal global variables */
int16_t TempInt;
int16_t TempIntOld;
int8_t DeltaTemp1;
int8_t DeltaTemp2;
uint16_t NTCInt;
uint8_t TimeAdjust = 35;
uint8_t Buttons;
uint8_t OldButtons;
uint8_t RotaryTemp;
uint8_t Rotary;
uint16_t RFLXCT;
uint16_t Position;
uint16_t ValveTop;
//uint16_t ValveBot;
uint8_t MotTimeOut;
uint8_t AdaptStep;
uint8_t MotorCurrent;
uint8_t FreeMotorCurrent;
uint8_t ADCPrescaler; // number of timer ticks (0.25 s) till next temperature readout
struct time TOD, Urlaub;
uint8_t UserDisplay;
uint16_t iNTCV;
uint8_t PSC1 = 4;
uint16_t SetTemp = 200;
int16_t TempOffset;
uint16_t Ek;
int8_t RegWay;
uint8_t DisplayCT;
uint8_t BeepCT;
uint8_t BeepLen;
uint8_t BeepPulseCT;
uint8_t BeepPulseLen;
uint8_t MenuLow;
uint8_t MenuHigh;
uint8_t DispTimer;
uint8_t OldBarBlink;
uint8_t FuzzyVal;
//uint8_t FuzzyCounter;
uint8_t test;

enum
{
    MANU, AUTO
}
OpMode;

enum automode
{
    OFF, INHOUSE, OFFHOUSE, NIGHT, WINDOW
};

enum automode CurrAutoMode;

uint16_t InHouseTemp = 220;
uint16_t OffHouseTemp = 190;
uint16_t NightTemp = 100;
uint16_t WindowOpenTemp;

uint8_t DisplayBuffer1[20];
uint8_t DisplayBuffer2[20];

uint8_t DailyTimer[TIMPERDAY * NTIMERS];

const __flash uint16_t NTCIntTable[] =
{
    34090, /*   0°C */
    26310, /*   5°C */
    20440, /*  10°C */
    16000, /*  15°C */
    12610, /*  20°C */
    10000, /*  25°C */
    7981, /*  30°C */
    6408, /*  35°C */
    5174, /*  40°C */
    4202, /*  45°C */
    3431, /*  50°C */
    2816, /*  55°C */
    2322, /*  60°C */
    1925, /*  65°C */
    1603, /*  70°C */
    1340, /*  75°C */
    1126, /*  80°C */
    949, /*  85°C */
    804, /*  90°C */
    684, /*  95°C */
    0
};

static bool Menu_Adapt(uint8_t);
static bool Menu_AdaptSub1(uint8_t);
static bool Menu_Dbg_1(uint8_t);
static bool Menu_Dbg_2(uint8_t);
static bool Menu_Dbg_3(uint8_t);
static bool Menu_Dbg_4(uint8_t);
static bool Menu_Dbg_5(uint8_t);
static bool Menu_Dbg_FW(uint8_t);
static bool Menu_Debug(uint8_t);
static bool Menu_Fens(uint8_t);
static bool Menu_FensSub1(uint8_t);
static bool Menu_FensSub11(uint8_t);
static bool Menu_FensSub2(uint8_t);
static bool Menu_FensSub3(uint8_t);
static bool Menu_Inst(uint8_t);
static bool Menu_InstSub1(uint8_t);
static bool Menu_Mode(uint8_t);
static bool Menu_ModeSub1(uint8_t);
static bool Menu_ModeSub11(uint8_t);
static bool Menu_ModeSub2(uint8_t);
static bool Menu_ModeSub21(uint8_t);
static bool Menu_Offs(uint8_t);
static bool Menu_OffsSub1(uint8_t);
static bool Menu_Prog(uint8_t);
static bool Menu_ProgSub1(uint8_t);
static bool Menu_ProgSub11(uint8_t);
static bool Menu_ProgSub12(uint8_t);
static bool Menu_ProgSub13(uint8_t);
static bool Menu_ProgSub14(uint8_t);
static bool Menu_ProgSub2(uint8_t);
static bool Menu_ProgSub3(uint8_t);
static bool Menu_ProgSub4(uint8_t);
static bool Menu_ProgSub5(uint8_t);
static bool Menu_ProgSub6(uint8_t);
static bool Menu_ProgSub7(uint8_t);
static bool Menu_ProgSub8(uint8_t);
static bool Menu_ProgSub9(uint8_t);
static bool Menu_ProgSubA(uint8_t);
static bool Menu_ProgSubB(uint8_t);
static bool Menu_Reset(uint8_t);
static bool Menu_ResetSub1(uint8_t);
static bool Menu_ResetSub2(uint8_t);
static bool Menu_Temp(uint8_t);
static bool Menu_TempSub1(uint8_t);
static bool Menu_TempSub2(uint8_t);
static bool Menu_TempSub3(uint8_t);
static bool Menu_Urla(uint8_t);
static bool Menu_UrlaSub1(uint8_t);
static bool Menu_UrlaSub2(uint8_t);
static bool Menu_UrlaSub3(uint8_t);
static bool Menu_UrlaSub4(uint8_t);
static bool Menu_Zeit(uint8_t);
static bool Menu_ZeitSub1(uint8_t);
static bool Menu_ZeitSub2(uint8_t);
static bool Menu_ZeitSub3(uint8_t);
static bool Menu_ZeitSub4(uint8_t);
static bool Menu_ZeitSub5(uint8_t);

typedef bool (*menuFunc)(uint8_t);

struct menuentry
{
    uint8_t main;
    uint8_t sub;
    menuFunc func;
};

const __flash struct menuentry MenuTable[] =
{
    { .main = 0x00, .sub = 0x00, .func = Menu_Mode      }, // MODE   ID0
    { .main = 0x00, .sub = 0x10, .func = Menu_ModeSub1  }, // MANU   ID1
    { .main = 0x00, .sub = 0x11, .func = Menu_ModeSub11 }, // MEnt   ID2
    { .main = 0x00, .sub = 0x20, .func = Menu_ModeSub2  }, // AUTO   ID3
    { .main = 0x00, .sub = 0x21, .func = Menu_ModeSub21 }, // AEnt   ID4
    { .main = 0x01, .sub = 0x00, .func = Menu_Prog      }, // PROG   ID5
    { .main = 0x01, .sub = 0x10, .func = Menu_ProgSub1  }, // TAG1   ID6
    { .main = 0x01, .sub = 0x11, .func = Menu_ProgSub11 },
    { .main = 0x01, .sub = 0x12, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x13, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x14, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x15, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x16, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x17, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x18, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x19, .func = Menu_ProgSub14 },
    { .main = 0x01, .sub = 0x20, .func = Menu_ProgSub2  }, // TAG2
    { .main = 0x01, .sub = 0x21, .func = Menu_ProgSub11 },
    { .main = 0x01, .sub = 0x22, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x23, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x24, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x25, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x26, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x27, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x28, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x29, .func = Menu_ProgSub14 },
    { .main = 0x01, .sub = 0x30, .func = Menu_ProgSub3  }, // TAG3
    { .main = 0x01, .sub = 0x31, .func = Menu_ProgSub11 },
    { .main = 0x01, .sub = 0x32, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x33, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x34, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x35, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x36, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x37, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x38, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x39, .func = Menu_ProgSub14 },
    { .main = 0x01, .sub = 0x40, .func = Menu_ProgSub4  }, // TAG4
    { .main = 0x01, .sub = 0x41, .func = Menu_ProgSub11 },
    { .main = 0x01, .sub = 0x42, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x43, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x44, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x45, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x46, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x47, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x48, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x49, .func = Menu_ProgSub14 },
    { .main = 0x01, .sub = 0x50, .func = Menu_ProgSub5  }, // TAG5
    { .main = 0x01, .sub = 0x51, .func = Menu_ProgSub11 },
    { .main = 0x01, .sub = 0x52, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x53, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x54, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x55, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x56, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x57, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x58, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x59, .func = Menu_ProgSub14 },
    { .main = 0x01, .sub = 0x60, .func = Menu_ProgSub6  }, // TAG6
    { .main = 0x01, .sub = 0x61, .func = Menu_ProgSub11 },
    { .main = 0x01, .sub = 0x62, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x63, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x64, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x65, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x66, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x67, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x68, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x69, .func = Menu_ProgSub14 },
    { .main = 0x01, .sub = 0x70, .func = Menu_ProgSub7  }, // TAG7
    { .main = 0x01, .sub = 0x71, .func = Menu_ProgSub11 },
    { .main = 0x01, .sub = 0x72, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x73, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x74, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x75, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x76, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x77, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x78, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x79, .func = Menu_ProgSub14 },
    { .main = 0x01, .sub = 0x80, .func = Menu_ProgSub8  }, // T1-5
    { .main = 0x01, .sub = 0x81, .func = Menu_ProgSub11 },
    { .main = 0x01, .sub = 0x82, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x83, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x84, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x85, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x86, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x87, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x88, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x89, .func = Menu_ProgSub14 },
    { .main = 0x01, .sub = 0x90, .func = Menu_ProgSub9  }, // T1-6
    { .main = 0x01, .sub = 0x91, .func = Menu_ProgSub11 },
    { .main = 0x01, .sub = 0x92, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x93, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x94, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x95, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x96, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x97, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0x98, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0x99, .func = Menu_ProgSub14 },
    { .main = 0x01, .sub = 0xA0, .func = Menu_ProgSubA  }, // T1-7
    { .main = 0x01, .sub = 0xA1, .func = Menu_ProgSub11 },
    { .main = 0x01, .sub = 0xA2, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0xA3, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0xA4, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0xA5, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0xA6, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0xA7, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0xA8, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0xA9, .func = Menu_ProgSub14 },
    { .main = 0x01, .sub = 0xB0, .func = Menu_ProgSubB  }, // T6-7
    { .main = 0x01, .sub = 0xB1, .func = Menu_ProgSub11 },
    { .main = 0x01, .sub = 0xB2, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0xB3, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0xB4, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0xB5, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0xB6, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0xB7, .func = Menu_ProgSub13 },
    { .main = 0x01, .sub = 0xB8, .func = Menu_ProgSub12 },
    { .main = 0x01, .sub = 0xB9, .func = Menu_ProgSub14 },
    { .main = 0x02, .sub = 0x00, .func = Menu_Temp      }, // TEMP   ID64
    { .main = 0x02, .sub = 0x10, .func = Menu_TempSub1  }, // InHouse
    { .main = 0x02, .sub = 0x20, .func = Menu_TempSub2  }, // OffHouse
    { .main = 0x02, .sub = 0x30, .func = Menu_TempSub3  }, // Night
    { .main = 0x03, .sub = 0x00, .func = Menu_Zeit      }, // ZEIT   ID67
    { .main = 0x03, .sub = 0x10, .func = Menu_ZeitSub1  }, // set year   ID68
    { .main = 0x03, .sub = 0x20, .func = Menu_ZeitSub2  }, // set month  ID69
    { .main = 0x03, .sub = 0x30, .func = Menu_ZeitSub3  }, // set date   ID70
    { .main = 0x03, .sub = 0x40, .func = Menu_ZeitSub4  }, // set hour   ID71
    { .main = 0x03, .sub = 0x50, .func = Menu_ZeitSub5  }, // set minute ID72
    { .main = 0x04, .sub = 0x00, .func = Menu_Fens      }, // FENS       ID73
    { .main = 0x04, .sub = 0x10, .func = Menu_FensSub1  }, // HOCH       ID74
    { .main = 0x04, .sub = 0x11, .func = Menu_FensSub11 }, // time resume ID75
    { .main = 0x04, .sub = 0x20, .func = Menu_FensSub2  }, // MITT       ID76
    { .main = 0x04, .sub = 0x21, .func = Menu_FensSub11 }, // time resume ID77
    { .main = 0x04, .sub = 0x30, .func = Menu_FensSub3  }, // NIED       ID78
    { .main = 0x04, .sub = 0x31, .func = Menu_FensSub11 }, // time resume ID79
    { .main = 0x05, .sub = 0x00, .func = Menu_Reset     }, // RESET      ID80
    { .main = 0x05, .sub = 0x10, .func = Menu_ResetSub1 }, // OK         ID81
    { .main = 0x05, .sub = 0x20, .func = Menu_ResetSub2 }, // DONE
    { .main = 0x06, .sub = 0x00, .func = Menu_Adapt     }, // ADAP       ID82
    { .main = 0x06, .sub = 0x10, .func = Menu_AdaptSub1 }, // ADAP       ID83
    { .main = 0x07, .sub = 0x00, .func = Menu_Urla      }, // URLA       ID84
    { .main = 0x07, .sub = 0x10, .func = Menu_UrlaSub1  },
    { .main = 0x07, .sub = 0x20, .func = Menu_UrlaSub2  },
    { .main = 0x07, .sub = 0x30, .func = Menu_UrlaSub3  },
    { .main = 0x07, .sub = 0x40, .func = Menu_UrlaSub4  },
    { .main = 0x08, .sub = 0x00, .func = Menu_Inst      }, // INST       ID88
    { .main = 0x08, .sub = 0x10, .func = Menu_InstSub1  }, // <<<<       ID89
    { .main = 0x09, .sub = 0x00, .func = Menu_Offs      }, // OFFS       ID90
    { .main = 0x09, .sub = 0x10, .func = Menu_OffsSub1  }, // set temp   ID91
    { .main = 0x0A, .sub = 0x00, .func = Menu_Debug     }, // DBUG       ID92
    { .main = 0x0A, .sub = 0x10, .func = Menu_Dbg_FW    }, // FW         ID93
    { .main = 0x0A, .sub = 0x20, .func = Menu_Dbg_1     }, // Fuzz       ID94
    { .main = 0x0A, .sub = 0x30, .func = Menu_Dbg_2     }, // Olim       ID95
    { .main = 0x0A, .sub = 0x40, .func = Menu_Dbg_3     }, // VTop
    { .main = 0x0A, .sub = 0x50, .func = Menu_Dbg_4     }, // RWay
    { .main = 0x0A, .sub = 0x60, .func = Menu_Dbg_5     }, // DTemp

    { .main = 0xFF, .sub = 0xFF, .func = 0},   // menu end
};

struct eedata eemem EEMEM;

static void Clear_Screen(void);
static void PutString(const __flash char *s);
static void PutFormatted(const __flash char *fmt, ...);
static void ReadButtons(void);
static void Adaptation(void);
static void Measure_Motor_Current(void);
static void Store_Time(void);
static void Store_Valvestate(void);
static void ReadBack_Time(void);
static void ReadBack_Progdata(void);
static void ReadBack_Valvestate(void);
static void Show_Current_Temperature(void);
static void Show_Current_Time(void);
static void User_Action(void);
static void Calc_Temperature(void);
static void MotorControl(void);
static void Clock(void);
static void Copy_DisplayBuffers(void);
static uint8_t Soft_SPI(uint8_t);
static void StartMain(void);
static void Regulate(void);
static void PutBargraph(uint8_t, uint8_t);
static void Clear_Bargraph(void);
static uint8_t CalcDayOfWeek(struct time *clock);
static uint8_t MonthLastDay(struct time *clock);
static void PutSymbol(uint8_t pos, uint8_t buffno);
static void ClearWeekDays(void);
static void PutWeekDay(uint8_t pos, uint8_t buffno);

/* --- From LCDTable.inc ----------------------------------------- */

// Segment macro, LCDDRx.y
#define S(x, y) ((x) * 8 + (y))

// LCD alphanumeric character segment bit position
const __flash uint8_t LCD_Segment_Table[4][14] =
{
    //left Digit
    {
        //a      b         c        d        e       f        g1
        S(15,6), S(15,4),  S(5,4),  S(0,5),  S(0,7), S(15,7), S(5,7),
        //g2     h         i        j        k       l        m
        S(10,5), S(10,7),  S(10,6), S(15,5), S(5,5), S(5,6),  S(0,6),
    },
    //middle left Digit
    {
        //a      b         c        d        e       f        g1
        S(15,3), S(15,1),  S(0,1),  S(0,2),  S(0,4), S(10,4), S(5,3),
        //g2     h         i        j        k       l        m
        S(10,1), S(10,3),  S(10,2), S(15,2), S(5,1), S(5,2),  S(0,3),
    },
    //middle right Digit
    {
        //a      b         c        d        e       f        g1
        S(17,1), S(17,3),  S(7,3),  S(2,2),  S(2,0), S(17,0), S(7,0),
        //g2     h         i        j        k       l        m
        S(12,2), S(12,0),  S(12,1), S(17,2), S(7,2), S(7,1),  S(2,1),
    },
    //right Digit
    {
        //a      b         c        d        e       f        g1
        S(17,4), S(17,6),  S(2,6),  S(2,5),  S(2,3), S(12,3), S(7,4),
        //g2     h         i        j        k       l        m
        S(12,6), S(12,4),  S(12,5), S(17,5), S(7,6), S(7,5),  S(2,4),
    }
    ,
};

// LCD alphanumeric chracter bitmap
const __flash uint16_t LCD_Character_Table[] =
{
    //              21
    //  mlkjihggfedcba
    0b0010010000111111,  // 0
    0b0000010000000110,  // 1
    0b0000000011011011,  // 2
    0b0000000010001111,  // 3
    0b0000000011100110,  // 4
    0b0000000011101101,  // 5
    0b0000000011111101,  // 6
    0b0001010000000001,  // 7
    0b0000000011111111,  // 8
    0b0000000011101111,  // 9
    0b0000000011000000,  // :  used as '-'
    0b0010010000001110,  // ;  used as Delta
    0b0000110000000000,  // <
    0b0001001011000000,  // =  used as '+'
    0b0001000100000000,  // >
    0b0000000000000000,  // ?  used as 'SPACE'
    0b0000000011100011,  // @  used as '°' character
    0b0000000011110111,  // A
    0b0001001010001111,  // B
    0b0000000000111001,  // C
    0b0001001000001111,  // D
    0b0000000001111001,  // E
    0b0000000001110001,  // F
    0b0000000010111101,  // G
    0b0000000011110110,  // H
    0b0001001000001001,  // I
    0b0000000000011110,  // J
    0b0000110001110000,  // K
    0b0000000000111000,  // L
    0b0000010100110110,  // M
    0b0000100100110110,  // N
    0b0000000000111111,  // O
    0b0000000011110011,  // P
    0b0000100000111111,  // Q
    0b0000100011110011,  // R
    0b0000000011101101,  // S
    0b0001001000000001,  // T
    0b0000000000111110,  // U
    0b0010010000110000,  // V
    0b0010100000110110,  // W
    0b0010110100000000,  // X
    0b0001010100000000,  // Y
    0b0010010000001001,  // Z
    //  mlkjihggfedcba
};

// Bargraph segment bit position
const __flash uint8_t Bargraph_Table[] =
{
    S(11,0), S(6,0), S(1,0), S(1,1), S(6,1), S(11,1), S(11,2), S(6,2),
    S(1,2), S(1,3), S(6,3), S(11,3), S(11,4), S(6,4), S(1,4), S(1,5),
    S(6,5), S(11,5), S(16,6), S(11,6), S(11,6), S(6,6), S(1,6), S(1,7), S(6,7),
};

// Week day bit position
const __flash uint8_t WDayTable[] =
{
    //Mo     Di       Mi       Do       Fr       Sa       So
    S(16,0), S(16,1), S(16,2), S(16,3), S(16,4), S(16,5), S(11,7)
};

// Symbol bit position
const __flash uint8_t SymbolTable[] =
{
    //? AUTO    MANU     Batt.   Radio   InHouse OutHou. Moon     Snow     Therm.   Lock
    0, S(10,0), S(15,0), S(5,0), S(2,7), S(3,0), S(8,0), S(13,0), S(18,0), S(12,7), S(17,7)
};

/* --- From ISR.inc ---------------------------------------------- */

/* Sound interrupt */
ISR(TIMER0_COMP_vect)
{
    BeepPulseCT++;
    if (BeepPulseCT < 10)
    {
        MotorDDR &= ~Motor_Close;
        MotorPort &= ~Motor_Close;
        MotorDDR |= Motor_Open;
        MotorPort |= Motor_Open;
    }
    else if (BeepPulseCT == 10 || BeepPulseCT >= 20)
    {
        MotorDDR &= ~Motor_Close;
        MotorPort &= ~Motor_Close;
        MotorDDR &= ~Motor_Open;
        MotorPort &= ~Motor_Open;
    }
    else
    {
        MotorDDR |= Motor_Close;
        MotorPort |= Motor_Close;
        MotorDDR &= ~Motor_Open;
        MotorPort &= ~Motor_Open;
    }
    if (BeepPulseCT == 40)
    {
        BeepPulseCT = 0;
        if (BeepCT++ >= BeepLen)
            TIMSK0 &= ~(1<<OCIE0A);
    }
}

/* Reflex coupler pin change interrupt */
ISR(PCINT0_vect)
{
    if (PINE & _BV(0))
    {
        /* RapidShutOff */
        // DebugPort &= ~Debug1;
        PORTB &= ~_BV(7);
        PORTB &= ~_BV(0);
        Store_Time();
        Store_Valvestate();
        Clear_Screen();

        PutString(FSTR("OFF "));

        set_sleep_mode(SLEEP_MODE_PWR_SAVE);
        for (;;)
            sleep_mode();
    }

    if (Status0 & MotRun)
    {
        /* execute only if motor hardware is on */
        ActiveCT = 3;
        if ((LightSensPin & LightSens_Trans) == 0)
        {
            /* count only falling edge */
            if (Status0 & Adapt)
                /* decrement Reflex counter down to 0 */ /* ??? */
                Position++;

            /* decrement Reflex counter down to 0 */
            if (RFLXCT != 0)
                RFLXCT--;
        }
        MotTimeOut &= (TopLimit | BotLimit); /* clear motor timeout counter */
    }
}

/* Buttons pin change interrupt */
ISR(PCINT1_vect)
{
    LCDCRA = (1<<LCDEN) | (1<<LCDIE) | (1<<LCDAB); // enable LCD-Frame Interrupt
    ActiveCT = 3;
    // BacklightPort |= Backlight_On;
    ReadButtons();
}

ISR(LCD_vect)
{
    /* increment motor timeout counter each LCD frame */
    if ((MotTimeOut & ~(TopLimit | BotLimit)) != (uint8_t)~(TopLimit | BotLimit))
        MotTimeOut++;
}

/* SystemTime(), called with 4 Hz rate */
ISR(TIMER2_OVF_vect)
{
    DispTimer = (DispTimer + 1) & 3;
    if (DispTimer == 0)
    {
        /* used for alternating display content, seconds timing */
        DisplayCT++;
        if (UserDisplay != 0)
        {
            if (--UserDisplay == 0)
                DisplayCT = 0;
        }
    }
    if (++Prescaler1 == PSC1)
    {
        //_SysTSeconds:
        //DebugPort |= Debug1;
        Prescaler1 = 0;
        PSC1 = 4;           /* restore prescaler interval after time correction */
        Status1 |= SecondTick;
        if (!(Status0 & Adapt) &&
            --ADCPrescaler == 0)
        {
            ADCPrescaler = 100; // next temperature readout in 25 s
            Status1 |= NewTemp; // trigger temperature readout
        }
        if (ActiveCT != 0)
        {
            if (--ActiveCT == 0)
            {
                LCDCRA = (1<<LCDEN) | (0<<LCDIE)| (1<<LCDAB); /* Disable LCD-Frame-ISR */
                BacklightPort &= ~Backlight_On;
                // DebugPort &= ~Debug2;
            }
        }
        //DebugPort &= ~Debug1;
        //DebugPort &= ~Debug2;
    }
}

/* --- From Init.inc --------------------------------------------- */

void ioinit(void)
{
    //  (-)                          Sniffer    Debug1     Debug2     (+)
    //RotaryA  "OK"  "MENU"  "TIME"  MISO/USB2  MOSI/USB3  SCK/USB4  RotaryB
    DDRB = 0b00000010;
    PORTB = 0b11111101;
    //MotorA  MotorB  Backlight  SCL_H2  AIN1_H2  REFLX_on  RFLX_in  1M_0V
    DDRE = 0b00100100;
    PORTE = 0b00111000;
    //JTAG TDI  JTAG TDO  JTAG TMS  JTAG TCK  Int_NTC_on  MOT_Cur  Int_NTC  RFLX_ADC
    DDRF = 0b00000000;
    PORTF = 0b11110000;

    DIDR0 = 0b00001111;
    PRR = (1<<PRTIM1) | (1<<PRSPI) | (1<<PRUSART0); //disable Timer1, SPI and UART
    //setup LCD-Controller
    LCDCRB = (1<<LCDCS) | (3<<LCDMUX0) | (7<<LCDPM0);
    LCDFRR = (0<<LCDPS0) | (3<<LCDCD0); //64Hz @ 32768 Hz
    LCDCCR = (1<<LCDDC2) | (1<<LCDDC1) | (1<<LCDDC0) |
        (1<<LCDCC3) | (1<<LCDCC2) |
        (0<<LCDCC1) | (1<<LCDCC1); //Contrast setting: drivers 50% on, 3.25Vlcd
    LCDCRA = (1<<LCDEN) | (1<<LCDIE) | (1<<LCDAB); //enable LCD controller, LCD-Frame Interrupt, low power waveform
    ActiveCT = 3;
    //setup ADC
    ADMUX = 0b01000001; //AVcc as reference, channel2, iNTCon
    ADCSRA = 0b00010011; //Prescaler 8, 62.5kHz
    //DIDR0 = 0b00000111; // contradicts earlier setting above
    // setup PinChange for Buttons:
    PCMSK1 = 0b11110001;
    //setup PinChange for Reflex coupler / Battery remove alert:
    PCMSK0 = 0b00000011;
    EIMSK = (1<<PCIE1) | (1<<PCIE0); //enable pin change on PCINT15...8 // PCINT7...0
    TCCR0A = 0b00001001; //CTC, prescaler 1
    OCR0A = 3;
    //setup Timer2 for SystemTime
    TCCR2A = (0<<CS22)| (1<<CS21) | (1<<CS20); //Normal mode, prescaler 32 => 1024 Hz clock
    TIMSK2 = (1<<TOIE2); //Overflow interrupt T2_A enabled => 4 Hz interrupt rate
    ASSR = (1<<AS2);

    BacklightPort &= ~Backlight_On;
    //DebugPort &= ~Debug1;

    ReadBack_Time();
    ReadBack_Progdata();
    ReadBack_Valvestate();

    sei();
}

/* --- From Sparmatic_Comet.asm ---------------------------------- */


int main(void)
{
    ioinit();

    StartMain();

    for (;;)
    {
        Adaptation();

        if (Status0 & Adapt)
            Measure_Motor_Current();

        if (!(Status0 & Adapt))
        {
            Show_Current_Temperature();
            Show_Current_Time();
            User_Action();
        }

        Calc_Temperature();
        MotorControl();
        Clock();
        Copy_DisplayBuffers();
        //FullDebug();
        //DebugSniffer();
        //BargraphTest();

        if (Status2 & TX_SSPI)
        {
            SSPI_PORT &= ~SSPI_MOSI; /* generate low level to wake up external peripheral */
            SSPI_DDR |= SSPI_MOSI;
            _delay_us(0.5);

            Soft_SPI(SetTemp >> 8);
            Soft_SPI(SetTemp & 0xFF);
            Soft_SPI(TempInt >> 8);
            Soft_SPI(TempInt & 0xFF);
            Soft_SPI(FuzzyVal);
            Soft_SPI(Position >> 8);
            Soft_SPI(Position & 0xFF);
            Soft_SPI(RegWay);

            SSPI_DDR &= ~SSPI_MOSI;
            SSPI_PORT |= SSPI_MOSI; /* enable pullup, level on MOSI rises */

            Status2 &= ~TX_SSPI;
        }

        /* GoSleep: */
        if (ActiveCT == 0)
        {
            /*
             * read button state before entering sleep
             * any rotary pin is connected to GND, disable corresponding
             * pullup to save power (~150µA worst case)
             * leave button and SPI pullups on
             */
            PORTB = (PINB & 0b10000001) | 0b01111100;
        }
        // DebugPort &= ~Debug1;
        set_sleep_mode(SLEEP_MODE_PWR_SAVE);
        sleep_mode();
        /* re-enable rotary pullups */
        PORTB |= _BV(0);
        PORTB |= _BV(7);
        // DebugPort |= Debug1;
    }
}

static void Store_Time(void)
{
    eeprom_write_block(&TOD.Minutes, &eemem.tod, sizeof(eemem.tod));
}

static void Store_Valvestate(void)
{
    eeprom_write_word(&eemem.valvestate.position, Position);
    eeprom_write_word(&eemem.valvestate.valvetop, ValveTop);
}

static void ReadBack_Time(void)
{
    eeprom_read_block(&TOD.Minutes, &eemem.tod, sizeof(eemem.tod));
    if (TOD.Minutes == 255)
        // uninitialized EEPROM, clear TOD
        memset(&TOD, 0, sizeof(TOD));
}

static void ReadBack_Progdata(void)
{
    eeprom_read_block(DailyTimer, &eemem.dailytimer, sizeof(eemem.dailytimer));
    for (uint8_t tmrno = 0; tmrno < NTIMERS; tmrno++)
        if (DailyTimer[TIMPERDAY * tmrno] == 255)
            // EEPROM data for this timer unset
            memset(&DailyTimer[TIMPERDAY * tmrno], 0, TIMPERDAY);
    uint16_t temp = eeprom_read_word(&eemem.temperatures.inhouse);
    if (temp != 0xFFFF)
        InHouseTemp = temp;
    temp = eeprom_read_word(&eemem.temperatures.offhouse);
    if (temp != 0xFFFF)
        OffHouseTemp = temp;
    temp = eeprom_read_word(&eemem.temperatures.night);
    if (temp != 0xFFFF)
        NightTemp = temp;
    temp = eeprom_read_word(&eemem.temperatures.windowopen);
    if (temp != 0xFFFF)
        WindowOpenTemp = temp;
    temp = eeprom_read_word((uint16_t *)&eemem.temperatures.offset);
    if (temp != 0xFFFF)
        WindowOpenTemp = (int16_t)temp;
}

static void ReadBack_Valvestate(void)
{
    // Try reading back valve state from EEPROM.
    // If data have been found, the initial adaptation step
    // can be skipped.  Invalidate EEPROM data in that case,
    // so they are only trusted if actually saved before
    // shutdown.
    Position = eeprom_read_word(&eemem.valvestate.position);
    ValveTop = eeprom_read_word(&eemem.valvestate.valvetop);
    if (Position == 0xFFFF)
        Position = 0;
    else
        eeprom_write_word(&eemem.valvestate.position, 0xFFFF);
    if (ValveTop == 0xFFFF)
        ValveTop = 0;
    else
        eeprom_write_word(&eemem.valvestate.valvetop, 0xFFFF);
}


static void StartMain(void)
{
    BeepPulseLen = 5;
    BeepLen = 200;
    TIMSK0 |= (1<<OCIE0A);
    while (TIMSK0 & (1<<OCIE0A)) {}
    // start Adaptation unless current valvedata are available
    if (ValveTop == 0)
    {
        FreeMotorCurrent = 0xFF;
        Position = 0;
        AdaptStep = 0;
        Status0 |= Adapt;
    }
    else
    {
        ADCPrescaler = 3; // next temperature readout in 0.75 s
        UserDisplay = 0;
        DisplayCT = 0;
        DispTimer = 0;
        uint8_t dow = CalcDayOfWeek(&TOD);
        ClearWeekDays();
        PutWeekDay(dow | 0x80, 3);
        DisplayBuffer1[0] |= 1; // switch on hour bar
        DisplayBuffer2[0] |= 1;
    }
    OpMode = MANU;
    PutSymbol(LCD_Manu_SET, 3);
    PutSymbol(LCD_Auto_CLR, 3);
}


/*
 * ----------------------------------------------------------------------------------------
 * Calculate temperatures from internal NTC voltage divider using the formula: R2=R1*U2/U1
 * ----------------------------------------------------------------------------------------
 */
static void Calc_Temperature(void)
{
    if (!(Status1 & NewTemp))
        return;

    Internal_NTC_DDR |= Internal_NTC_On;
    Internal_NTC_Port |= Internal_NTC_On; /* switch NTC voltage divider on */

    ADMUX = 0b01000011; /* AVcc as reference, channel 3, internal NTC */
    _delay_us(0.5);
    ADCSRA = 0b11000100; /*ADC enable, start ADC conversion, ADC Prescaler 16 */
    while ((ADCSRA & _BV(ADSC)) != 0)
    {
        /* measure upper voltage on voltage divider (U1) */
    }
    iNTCV = ADCW;               /* store U1 */

    ADMUX = 0b01000001; /* AVcc as reference, channel 3, internal NTC */ /* ??? */
    _delay_us(0.5);
    ADCSRA = 0b11000100; /*ADC enable, start ADC conversion, ADC Prescaler 16 */
    while ((ADCSRA & _BV(ADSC)) != 0)
    {
        /* measure NTC voltage */
    }
    NTCInt = ADCW;

    Status1 &= ~NewTemp;

    Internal_NTC_DDR &= ~Internal_NTC_On;
    Internal_NTC_Port &= ~Internal_NTC_On; /* switch NTC voltage divider off */

    ADCSRA = 0b00000100; /* ADC disable, ADC Prescaler 16 */

    // U2 / U1
    uint16_t ratio = (NTCInt * 12000ul) / (iNTCV - NTCInt);
    if (NTCIntTable[0] < ratio)
    {
        // if table value is lower then read value, set temp to 0.00°C and skip
        TempInt = 0;
    }
    else
    {
        TempInt = 0;
        for (int i = 1;; i++)
        {
            if (NTCIntTable[i] == 0)
            {
                //TempTop
                TempInt = 0xFFFF;  // ??? -> 0xFFFF is negative
                break;
            }
            if (NTCIntTable[i] < ratio)
            {
                // if table value is lower then read value, skip to fine reolution
                uint16_t step = NTCIntTable[i - 1] - NTCIntTable[i];
                uint16_t delta = NTCIntTable[i - 1] - ratio;
                TempInt += delta / (step / 50); // divide values with 50, 0.1° resolution
                break;
            }
            TempInt += 50;
        }
    }
    if (!(Status0 & Adapt))
        Regulate();
    Status2 |= TX_SSPI;
}

/* --- From Regulation.inc --------------------------------------- */

void OpenValve(uint8_t amount)
{
    RegWay = 0;
    // limit position change to ValveTop position
    if (Position == ValveTop)
    {
        return;
    }
    if (Position + amount > ValveTop)
    {
        amount = ValveTop - Position;
        Position = ValveTop;
    }
    else
    {
        Position += amount;
    }
    if (MotTimeOut & TopLimit)
        return;
    RegWay = amount;
    Status0 &= ~MotDir; // open valve
    if (Status0 & MotRun)
        // do not store new motor position, if motor is running
        return;
    Status0 |= MotOn;
    MotTimeOut &= (TopLimit | BotLimit);
    RFLXCT = amount;
}


void CloseValve(uint8_t amount)
{
    RegWay = 0;
    if (amount > Position)
    {
        Position = 0;
    }
    else
    {
        Position -= amount;
    }
    // ignore closing the valve, if valve is fully closed
    if (MotTimeOut & BotLimit)
        return;
    RegWay = -amount;
    Status0 |= MotDir; // close valve
    if (Status0 & MotRun)
        // do not store new motor position, if motor is running
        return;
    Status0 |= MotOn;
    MotTimeOut &= (TopLimit | BotLimit);
    RFLXCT = amount;
}


void Regulate(void)
{
    int16_t t = TempInt;   // NTC temperature
    int16_t to = TempIntOld;
    TempIntOld = TempInt;  // store current temperature as old for next run
    DeltaTemp2 = DeltaTemp1; // mov previous Delta1 to Delta2
    DeltaTemp1 = t - to; // get new Delta1
    uint8_t t3;
    if (DeltaTemp1 == 0) t3 = TStatic;
    else if (DeltaTemp1 < 0) t3 = TFall;
    else t3 = TRise;
    int8_t dt = DeltaTemp1 + DeltaTemp2; // dt: Di0 in asm code
    if (dt < 0) dt = -dt;
    if (dt == 0) dt = 1;
    if (dt > 4) dt = 4;
    int16_t diff = TempInt + TempOffset - SetTemp;

    if (diff >= 15)
    {
        FuzzyVal = FuzzAbove + t3;
        switch (t3)
        {
            case TStatic:
            case TFall:
                CloseValve(50);
                break;
            case TRise:
                CloseValve(100);
                break;
        }
    }
    else if (diff >= 6)
    {
        FuzzyVal = FuzzHot + t3;
        switch (t3)
        {
            case TStatic:
                CloseValve(10);
                break;
            case TFall:
                RegWay = 0;
                return;
            case TRise:
                CloseValve(20 * dt);
                break;
        }
    }
    else if (diff >= 2)
    {
        FuzzyVal = FuzzWarm + t3;
        switch (t3)
        {
            case TStatic:
                CloseValve(3);
                break;
            case TFall:
                OpenValve(10 * dt);
                break;
            case TRise:
                CloseValve(10 * dt);
                break;
        }
    }
    else if (diff <= -2)
    {
        FuzzyVal = FuzzCool + t3;
        switch (t3)
        {
            case TStatic:
                OpenValve(3);
                break;
            case TFall:
                OpenValve(10 * dt);
                break;
            case TRise:
                CloseValve(10 * dt);
                break;
        }
    }
    else if (diff <= -6)
    {
        FuzzyVal = FuzzCold + t3;
        switch (t3)
        {
            case TStatic:
                OpenValve(10);
                break;
            case TFall:
                OpenValve(20 * dt);
                break;
            case TRise:
                RegWay = 0;
                return;
        }
    }
    else if (diff <= -15)
    {
        FuzzyVal = FuzzBelow + t3;
        switch (t3)
        {
            case TStatic:
            case TRise:
                OpenValve(50);
                break;
            case TFall:
                OpenValve(100);
                break;
        }
    }
    else /* -1 ... 1 */
    {
        // nothing to do now
        FuzzyVal = FuzzOK + t3;
        RegWay = 0;
        return;
    }
}

/* --- From LCD_Driver.inc --------------------------------------- */

void Clear_Bargraph(void)
{
    DisplayBuffer1[1] = 0;
    DisplayBuffer2[1] = 0;
    DisplayBuffer1[6] = 0;
    DisplayBuffer2[6] = 0;
    DisplayBuffer1[11] &= 0x80;
    DisplayBuffer2[11] &= 0x80;
    DisplayBuffer1[16] &= ~0x40;
    DisplayBuffer2[16] &= ~0x40;
}

void Clear_Screen(void)
{
    memset(DisplayBuffer1, 0, 40);
}

void Show_TimerSetBar(uint8_t *BarBase, uint8_t *set_time)
{
    Clear_Bargraph();
    uint8_t *p;
    uint8_t i;
    // number of inhouse/offhouse timers
    for (i = 0, p = BarBase; i < 4; i++)
    {
        uint8_t hr = *p++ / 6;
        for (uint8_t h = 0; h < 24; h++)
        {
            if (h == hr)
            {
                // set Barpoint only, if off time is later then on time
                if (p[0] > p[-1])
                {
                    PutBargraph(h | 0x80, 3);
                    uint8_t h1 = (*p++ - 1) / 6 + 1;
                    while (h1 != h)
                    {
                        PutBargraph(h | 0x80, 3);
                        if (++h >= 24)
                            break;
                    }
                    h = 24; // force next inhouse/offhouse timer
                }
                else
                {
                    p++;
                }
            }
        }
    }
    if (set_time)
    {
        uint8_t t = *set_time / 6;
        PutBargraph(t, 2);
        PutBargraph(t | 0x80, 1);
    }
}

void Show_Current_Temperature(void)
{
    if (Status0 & MenuOn)
        // don't show current temperature in menu mode
        return;

    if (UserDisplay == 0)
    {
        // in normal display mode show current temperature
        if (DispTimer != 0)
            // update display only once
            return;
        if ((DisplayCT & 0b000000111) != 0)
            return;
        PutFormatted(FSTR("%3d" DEGREE), TempInt + TempOffset);
    }
    else if (UserDisplay == 5)
    {
        // Show Set Temp
        PutFormatted(FSTR("%3d" DEGREE "\n    "), SetTemp);
        // clear point between segment 1 and 2
        DisplayBuffer1[7] &= ~0x80;
        DisplayBuffer2[7] &= ~0x80;
    }
    else
    {
        return;
    }
    // clear double point between segment 1 and 2
    DisplayBuffer1[16] &= ~0x80;
    DisplayBuffer2[16] &= ~0x80;
    // set point between segment 1 and 2
    DisplayBuffer1[7] |= 0x80;
    DisplayBuffer2[7] |= 0x80;
}

void Show_Current_Time(void)
{
    if (Status0 & MenuOn)
        // don't show current time in menu mode
        return;

    if (UserDisplay != 0)
        // show current time in normal display mode only
        return;

    if (DispTimer != 0)
        // update display only once
        return;

    if ((DisplayCT & 0b000000111) != 4)
        return;

    // clear point between segment 1 and 2
    DisplayBuffer1[7] &= ~0x80;
    DisplayBuffer2[7] &= ~0x80;

    PutFormatted(FSTR("%2d%02d"), TOD.Hours, TOD.Minutes);
    // set colon between segment 1 and 2
    DisplayBuffer1[16] |= 0x80;
}

static void Get_Character(uint8_t *bp, char c, uint8_t pos)
{
    if (pos >= 4)
        return;
    if (c < '0' || c > 'Z')
        return;
    uint16_t bits = LCD_Character_Table[(unsigned)(c - '0')];
    for (uint8_t i = 0; i < 14; i++, bits >>= 1)
    {
        uint8_t byteno = LCD_Segment_Table[pos][i] >> 3;
        uint8_t bitpos = 1 << (LCD_Segment_Table[pos][i] & 7);
        if (bits & 1)
            bp[byteno] |= bitpos;
        else
            bp[byteno] &= ~bitpos;
    }
}

static char MapChar(char c)
{
    if (c == ' ') return '?';
    if (c == '+') return '=';
    if (c == '-') return ':';
    return c;
}


static void PutStringBackend(const char *s, bool is_flash)
{
    // two lines given?
    const char *x = is_flash? strchr_P(s, '\n'): strchr(s, '\n');
    if (x)
    {
        // yes: fill both lines into different display buffers
        uint8_t pos = 0;
        const char *cp;
        for (cp = s;; cp++, pos++)
        {
            unsigned char c = is_flash? pgm_read_byte(cp): *cp;
            if (c == '\n')
                break;
            Get_Character(DisplayBuffer1, MapChar(c), pos);
        }
        cp++;
        pos = 0;
        for (;; cp++, pos++)
        {
            unsigned char c = is_flash? pgm_read_byte(cp): *cp;
            if (c == 0)
                break;
            Get_Character(DisplayBuffer2, MapChar(c), pos);
        }
    }
    else
    {
        // no: fill both buffers with same contents
        uint8_t pos = 0;
        for (const char *cp = s;; cp++, pos++)
        {
            unsigned char c = is_flash? pgm_read_byte(cp): *cp;
            if (c == 0)
                break;
            c = MapChar(c);
            Get_Character(DisplayBuffer1, c, pos);
            Get_Character(DisplayBuffer2, c, pos);
        }
    }
}


void PutString(const __flash char *s)
{
    PutStringBackend((const char *)s, true);
}

void PutFormatted(const __flash char *fmt, ...)
{
    char b[20];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf_P(b, sizeof(b), (const char *)fmt, ap);
    va_end(ap);

    PutStringBackend(b, false);
}

void Copy_DisplayBuffers(void)
{
    memcpy((void *)&LCDDR0,
        (DispTimer & 2)? DisplayBuffer2: DisplayBuffer1,
        sizeof(DisplayBuffer1));
}

void WriteSymbol(uint8_t *bp, const __flash uint8_t *p, uint8_t pos)
{
    uint8_t sym = p[pos & 0x1f]; // sym = x * 8 + y
    uint8_t byteno = sym >> 3;   // x
    uint8_t bitpos = 1 << (sym & 7); // 1<<y
    if (pos & 0x80)
        bp[byteno] |= bitpos;
    else
        bp[byteno] &= ~bitpos;
}


void OutSymbols(const __flash uint8_t *p, uint8_t pos, uint8_t buffno)
{
    if (buffno & 1)
        WriteSymbol(DisplayBuffer1, p, pos);
    if (buffno & 2)
        WriteSymbol(DisplayBuffer2, p, pos);
}


// write bargraph to LCD
// MSB of pos: set vs. clear bit
void PutBargraph(uint8_t pos, uint8_t buffno)
{
    OutSymbols(Bargraph_Table, pos, buffno);
}

// write Day of Week to LCD
// MSB of pos: set vs. clear bit
void PutWeekDay(uint8_t pos, uint8_t buffno)
{
    OutSymbols(WDayTable, pos, buffno);
}

// write Symbol to LCD
// MSB of pos: set vs. clear bit
void PutSymbol(uint8_t pos, uint8_t buffno)
{
    OutSymbols(SymbolTable, pos, buffno);
}

// set colon between segment 1 and 2
void SetColon(uint8_t buffno)
{
    if (buffno & 1)
        DisplayBuffer1[16] |= 0x80;
    if (buffno & 2)
        DisplayBuffer2[16] |= 0x80;
}

// clear colon between segment 1 and 2
void ClearColon(void)
{
    DisplayBuffer1[16] &= ~0x80;
    DisplayBuffer2[16] &= ~0x80;
}

// set point between segment 1 and 2
void SetPoint(void)
{
    DisplayBuffer1[7] |= 0x80;
    DisplayBuffer2[7] |= 0x80;
}

// clear point between segment 1 and 2
void ClearPoint(void)
{
    DisplayBuffer1[7] &= ~0x80;
    DisplayBuffer2[7] &= ~0x80;
}

void CopyWeekDays1_2(void)
{
    DisplayBuffer2[16] = (DisplayBuffer2[16] & 0xc0) |
        (DisplayBuffer1[16] & 0x3f);
    DisplayBuffer2[11] = (DisplayBuffer2[11] & 0x7f) |
        (DisplayBuffer1[11] & 0x80);
}

// clear all days of week on LCD
void ClearWeekDays(void)
{
    DisplayBuffer1[16] &= 0xc0;
    DisplayBuffer2[16] &= 0xc0;
    DisplayBuffer1[11] &= 0x7f;
    DisplayBuffer2[11] &= 0x7f;
}

void ClearSymbols(void)
{
    DisplayBuffer1[3] &= 0xfe; // InHouse
    DisplayBuffer2[3] &= 0xfe;
    DisplayBuffer1[8] &= 0xfe; // OffHouse
    DisplayBuffer2[8] &= 0xfe;
    DisplayBuffer1[13] &= 0xfe; // Moon
    DisplayBuffer2[13] &= 0xfe;
}

/* --- From Debug.inc -------------------------------------------- */

void BargraphTest(void)
{
    PutBargraph(test, 3);
    if ((++test & 0x7f) == 24)
        test = (test & 0x80) ^ 0x80;
}

/* --- From Motor.inc -------------------------------------------- */

void Adaptation(void)
{
    // execute in adaptation mode only
    if ((Status0 & Adapt) == 0)
        return;
    if (Status0 & MotDir)
    {
        // take motor current with no load on outer position - n
        if (RFLXCT == MaxAdaptWay - 20)
        {
            FreeMotorCurrent = MotorCurrent;
            // set bargraph point 1, freewheeling current measured
            DisplayBuffer1[11] |= 1;
            DisplayBuffer2[11] |= 1;
        }
        // threshold current over freewheeling current, to detect valve touch
        if (MotorCurrent >= FreeMotorCurrent + 10)
        {
            // execute only once if the current rises over threshold
            if ((DisplayBuffer1[6] & 1) == 0)
            {
                Position = 0;
                // set bargraph point 2, touch current measured
                DisplayBuffer1[6] |= 1;
                DisplayBuffer2[6] |= 1;
            }
            RFLXCT = 16; // keep Reflex counter well above 0
        }
    }
    if (Status0 & MotRun)
        return;
    switch (AdaptStep)
    {
        case 0:
            // AdpOpenValve
            Status0 &= ~MotDir;
            Status0 |= MotOn;
            MotTimeOut = 0;
            AdaptStep++;
            // flashing arrow "moving" left
            PutString(FSTR("< < \n < <"));
            // asm code sets only RFLXCTH (why?)
            RFLXCT = 0x1000;
            break;

        case 2:
            // WaitADP_OK
            if (Status0 & NewButton)
            {
                Status0 &= ~NewButton;
                if (Buttons == OK_Button)
                {
                    AdaptStep = 4;
                    PutString(FSTR("ADAP"));
                    Position = 0;
                }
            }
            break;

        case 4:
            // AdpCloseValve
            Status0 |= MotDir;
            Status0 |= MotOn;
            MotTimeOut = 0;
            AdaptStep++;
            RFLXCT = MaxAdaptWay; // preload maximum way without valve touch
            break;

        case 6:
            // AdpEnd
            AdaptStep = 0;
            DisplayBuffer1[11] &= ~1; // clear bargraph points 1 and 2
            DisplayBuffer2[11] &= ~1;
            DisplayBuffer1[6] &= ~1;
            DisplayBuffer2[6] &= ~1;
            ValveTop = Position - 80; // subtact margin to full open position
            Position = 0;
            ADCPrescaler = 3; // next temperature readout in 0.75 s
            UserDisplay = 0;
            DisplayCT = 0;
            DispTimer = 0;
            uint8_t dow = CalcDayOfWeek(&TOD);
            ClearWeekDays();
            PutWeekDay(dow | 0x80, 3);
            DisplayBuffer1[0] |= 1; // switch on hour bar
            DisplayBuffer2[0] |= 1;
            Status0 &= ~Adapt;
            break;
    }
}

void MotorControl(void)
{
    uint8_t tmo = MotTimeOut & ~(BotLimit | TopLimit);
    // timeout value to completely shut down motor hardware, if lower, execute motor control
    if (tmo >= 30)
    {
        Status0 &= ~MotRun;
        LightSensPort &= ~LightSens_LED; // switch off reflex sensor
        // if any valve limit is touched, clear reflex counter
        if (MotTimeOut & (BotLimit | TopLimit))
            RFLXCT = 0;
        return;
    }
    // exit, if motor has been already stopped
    if (!(Status0 & MotOn))
        return;
    // timeout value to shut down motor, if lower, execute motor control
    if (tmo >= 10)
    {
        // DebugPort |= Debug2;
        bool botlimit = false;
        if (Status0 & MotDir)
        {
            MotTimeOut = tmo | BotLimit;
            botlimit = true;
        }
        else
        {
            MotTimeOut = tmo | TopLimit;
        }
        if (botlimit && (Status0 & Adapt) == 0)
        {
            // if valve is fully closed, clear position counter
            Position = 0;
        }
    }
    // check if position is reached
    else if (RFLXCT != 0)
    {
        // execute only if motor hardware was stopped before
        if ((Status0 & MotRun) != 0)
            return;
        LightSensPort |= LightSens_LED; // enable reflex coupler
        // enable LCD-Frame Interrupt, used for upcounting Motor timeout
        LCDCRA = (1<<LCDEN) | (1<<LCDIE) | (1<<LCDAB);
        ActiveCT = 3;
        BacklightPort |= Backlight_On;
        // dependent on direction flag open or close valve
        if (Status0 & MotDir)
        {
            // MotorClose
            MotTimeOut &= ~TopLimit; // clear opposed limit flag
            // DebugPort &= ~Debug2;
            MotorDDR &= ~Motor_Open;
            MotorPort &= ~Motor_Open;
            _delay_us(0.25);
            MotorDDR |= Motor_Close;
            MotorPort |= Motor_Close;
            Status0 |= MotRun;
        }
        else
        {
            // Motor_Open
            MotTimeOut &= ~BotLimit; // clear opposed limit flag
            // DebugPort &= ~Debug2;
            MotorDDR &= ~Motor_Close;
            MotorPort &= ~Motor_Close;
            _delay_us(0.25);
            MotorDDR |= Motor_Open;
            MotorPort |= Motor_Open;
            Status0 |= MotRun;
        }
        return;
    }
    // MotorStop
    if (!(Status0 & MotOn))
        // if motor has been stopped before, exit
        return;
    MotorDDR &= ~Motor_Close;
    MotorPort &= ~Motor_Close; // disable both motor drivers
    MotorDDR &= ~Motor_Open;
    MotorPort &= ~Motor_Open;
    Status0 &= ~MotOn;
    if ((Status0 & Adapt) == 0)
        return;
    // in adaptation mode, increase adaptation step
    if (AdaptStep == 1)
    {
        PutString(FSTR("INST\n OK "));
    }
    AdaptStep++;
}

/* --- From User.inc --------------------------------------------- */

void ReadButtons(void)
{
    //                  PINB76543210
    //                  ------------
    //                      E TM   E
    //                      NOIE   N
    //                      CKMN   C
    //                      1 EU   2
    uint8_t pins = ~PINB & 0b11110001;
    // mask used pins and swap nibbles, put all button pins together -> 000bbbbb
    pins = (pins << 4) | (pins >> 4);
    // if no button is pressed, clear button value and status flag
    if ((Buttons = pins & (OK_Button | Menu_Button | Time_Button)) != 0)
        Status0 |= NewButton;
    else
        Status0 &= ~NewButton;
    // mask rotary pins
    pins &= 0b00011000;  // xxx12xxx
    pins >>= 3;          // xxxxxx12
    pins |= RotaryTemp;  // xxABCD12
    pins <<= 2;          // ABCD12xx
    // shift and store current rotary pin state
    pins &= 0b00111100;  // xxCD12xx
    RotaryTemp = pins;
    // check for first half step in clockwise direction
    if (pins == 0b00000100 || pins == 0b00111000)
    {
        // ROTStepCW1
        // mark first half step detected CW
        Rotary &= 0b00111111;
        Rotary |= 0b10000000;
    }
    // check for first half step in counter-clockwise direction
    else if (pins == 0b00001000 || pins == 0b00110100)
    {
        // ROTStepCCW1
        // mark first half step detected CCW
        Rotary &= 0b00111111;
        Rotary |= 0b01000000;
    }
    // if first half step was detected, check for second half step CW
    else if ((Rotary & 0b11000000) == 0b10000000)
    {
        // RotStepCW2
        // check for second half step CW
        if (pins == 0b00011100 || pins == 0b00100000)
        {
            // ROTStepCWP
            Rotary = 0b00000011; // set CW flag and new rotary flag
            // DebugPort |= Debug1;
        }
    }
    // if first half step was detected, check for second half step CCW
    else if ((Rotary & 0b11000000) == 0b01000000)
    {
        // RotStepCCW2
        // check for second half step CCW
        if (pins == 0b00101100 || pins == 0b00010000)
        {
            // ROTStepCCWP
            Rotary = 0b00000010; // set new rotary flag
            // DebugPort |= Debug2;
        }
    }
}

static void Set_SetTemperature_Up(void)
{
    if (SetTemp != 300) // max 30.0 °C
    {
        SetTemp += 5; // +0.5 K
        UserDisplay = 5;
    }
    ADCPrescaler = 10; // next temperature readout in 2.5 s
}

static void Set_SetTemperature_Down(void)
{
    if (SetTemp != 4) // min 4.0 °C
    {
        SetTemp -= 5; // -0.5 K
        UserDisplay = 5;
    }
    ADCPrescaler = 10; // next temperature readout in 2.5 s
}

static void EvalAutoMode(void)
{
    uint8_t time10min = TOD.Hours * 6 + TOD.Minutes / 10;
    uint8_t dow = TOD.WDays;
    // avoid out of bounds access due to wrong clock
    if (dow > 6) dow = 0;
    if (time10min > 24 * 6) time10min = 0;
    uint8_t *timerBase = DailyTimer + TIMPERDAY * dow;
    enum automode newMode = OFF;
    for (uint8_t i = 0; i < (TIMPERDAY / 2 - 1); i++)
    {
        if (time10min >= timerBase[2 * i] &&
            time10min < timerBase[2 * i + 1])
            // interval matches
        {
            newMode = INHOUSE;
            break;
        }
    }
    if (newMode == OFF)
    {
        // no iterval matched; if time10min is beyond
        // night limit, or before first value, set
        // night mode, otherwise offhouse mode
        if ((timerBase[8] != 0 && time10min >= timerBase[8]) ||
            time10min < timerBase[0])
            newMode = NIGHT;
        else
            newMode = OFFHOUSE;
    }
    if (newMode != CurrAutoMode)
    {
        // transition, set new temperature
        CurrAutoMode = newMode;
        ClearSymbols();
        switch (newMode)
        {
            case INHOUSE:
                if (Urlaub.Years && Urlaub.Months && Urlaub.Days)
                {
                    // vacation mode, restrict temperature to offhouse
                    PutSymbol(LCD_Case_SET, 3);
                    PutSymbol(LCD_OffHouse_SET, 3);
                    SetTemp = OffHouseTemp;
                }
                else
                {
                    PutSymbol(LCD_InHouse_SET, 3);
                    SetTemp = InHouseTemp;
                }
                break;

            case OFFHOUSE:
                if (Urlaub.Years && Urlaub.Months && Urlaub.Days)
                    // vacation mode
                    PutSymbol(LCD_Case_SET, 3);
                PutSymbol(LCD_OffHouse_SET, 3);
                SetTemp = OffHouseTemp;
                break;

            case NIGHT:
                if (Urlaub.Years && Urlaub.Months && Urlaub.Days)
                    // vacation mode
                    PutSymbol(LCD_Case_SET, 3);
                PutSymbol(LCD_Moon_SET, 3);
                SetTemp = NightTemp;
                break;

            default:
                // can't happen; no change
                break;
        }
    }
}


static void Get_Menu(uint8_t task)
{
    ActiveCT = 20;
    // BacklightPort |= Backlight_On;
    // if work menu is selected, skip new menu selection and go to current menu
    if (Status0 & MenuWork)
    {
        if (!menuID->func(task))
            return;
        // if the menufunction returned "true", re-enter search
        task = 0;
    }

    switch (task)
    {
        case 3:
            if ((MenuLow & 0xF0) == 0)
            {
                // IncSub
                // switch to SubLayer1
                MenuLow += 0x10;
            }
            else if ((MenuLow & 0x0F) == 0)
            {
                // IncSubSub
                // switch to SubLayer2
                MenuLow++;
            }
            else
            {
                return;
            }
            goto search;

        case 2:
            Status0 |= MenuUpDn;
            if ((MenuLow & 0xF0) == 0)
            {
                // IncMainMenu
                MenuHigh++;
            }
            else if ((MenuLow & 0x0F) == 0)
            {
                // IncSublayer1
                // increment Menu SubLayer1
                if (MenuLow == 0xF0)
                    MenuLow = 0x10;
                else
                    MenuLow += 0x10;
            }
            else
            {
                // IncSublayer2
                // DebugPort |= Debug2;
                // increment Menu SubLayer2
                if ((MenuLow & 0x0F) == 0xF)
                    MenuLow = (MenuLow & 0xF0) | 0x01;
                else
                    MenuLow = (MenuLow & 0xF0) | ((MenuLow + 1) & 0x0F);
            }
            goto search;

        case 1:
            Status0 &= ~MenuUpDn;
            if ((MenuLow & 0xF0) == 0)
            {
                // DecMainMenu
                MenuHigh--;
            }
            else if ((MenuLow & 0x0F) == 0)
            {
                // DecSublayer1
                // decrement Menu SubLayer1
                if (MenuLow == 0x10)
                    MenuLow = 0xF0;
                else
                    MenuLow -= 0x10;
            }
            else
            {
                // DecSublayer2
                // DebugPort &= ~Debug2;
                // decrement Menu SubLayer2
                if ((MenuLow & 0x0F) == 1)
                    MenuLow = (MenuLow & 0xF0) | 0x0F;
                else
                    MenuLow = (MenuLow & 0xF0) | ((MenuLow - 1) & 0x0F);
            }
            /* FALLTHROUGH */
        case 0:
        search:
            for (;;)
            {
                menuID = MenuTable;
                while (menuID->main != 0xFF)
                {
                    if (menuID->main == MenuHigh && menuID->sub == MenuLow)
                    {
                        // found
                        (void)menuID->func(0);
                        return;
                    }
                    menuID++;
                }
                // MenuEnd
                if (MenuLow == 0)
                {
                    // ME_Main
                    if (MenuHigh == 0xFF)
                    {
                        MenuHigh = 0x0A; // number of current main menues
                        MenuLow = 0;
                    }
                    else
                    {
                        MenuHigh = MenuLow = 0;
                    }
                    continue;
                }
                // if lower nibble is not 0, go to 2nd sub menu
                else if ((MenuLow & 0x0F) != 0)
                {
                    // ME_Sub2
                    // if subsubmenu is 1 and not found, exit
                    if ((MenuLow & 0x0F) == 1)
                    {
                        MenuLow = 0;
                        return;
                    }
                    // if search direction is up, add submenu and search
                    if (Status0 & MenuUpDn)
                    {
                        if ((MenuLow & 0x0F) == 0xF)
                            MenuLow = (MenuLow & 0xF0) | 0x01;
                        else
                            MenuLow = (MenuLow & 0xF0) | ((MenuLow + 1) & 0x0F);
                        continue;
                    }
                    // if search direction is down, sub submenu and search
                    MenuLow--;
                    // if subtracted down to 0 and no menu was found, exit
                    if ((MenuLow & 0x0F) == 0)
                    {
                        MenuLow = 0;
                        return;
                    }
                    continue;
                }
                else
                {
                    // ME_Sub1
                    // if submenu is 1 and not found, exit
                    if (MenuLow == 0x10)
                    {
                        MenuLow = 0;
                        return;
                    }
                    // if search direction is up, add submenu and search
                    if (Status0 & MenuUpDn)
                    {
                        if ((MenuLow += 0x10) == 0)
                            MenuLow = 0x10;
                        continue;
                    }
                    // if search direction is down, sub submenu and search
                    MenuLow -= 0x10;
                    continue;
                }
            }
            break;
    }
}

static void MenuModeOff(void)
{
    Status0 &= ~MenuOn;
    Status0 &= ~MenuWork;
    UserDisplay = 1;
    DispTimer = 3;
    ClearWeekDays();
    PutWeekDay(TOD.WDays | 0x80, 3);
    ClearSymbols();
    if (OpMode == AUTO)
    {
        CurrAutoMode = OFF;
        EvalAutoMode();
    }
}

void User_Action(void)
{
    // if new rotary value or new button press detected, execute
    if (Rotary & 2)
    {
        // UAC_Rotary
        Rotary &= ~2; // clear new rotary flag
        if (Rotary & 1)
        {
            // RotaryPlus
            if (Status0 & MenuOn)
            {
                Get_Menu(2);
            }
            else
            {
                Set_SetTemperature_Up();
                // NTC temperature  = OldTemperature
                TempIntOld = TempInt;
            }
        }
        else
        {
            // RotaryMinus
            if (Status0 & MenuOn)
            {
                Get_Menu(1);
            }
            else
            {
                Set_SetTemperature_Down();
                // NTC temperature  = OldTemperature
                TempIntOld = TempInt;
            }
        }
    }
    else if (Status0 & NewButton)
    {
        // UAC_Button
        Status0 &= ~NewButton;
        if (Buttons == Menu_Button)
        {
            // Toggle_Menu
            if (!(Status0 & MenuOn))
            {
                // MenuModeOn
                Status0 |= MenuOn;
                MenuHigh = MenuLow = 0;
                ClearColon();
                ClearPoint();
                Get_Menu(0);
            }
            else
            {
                MenuModeOff();
            }
        }
        else if (Buttons == OK_Button)
        {
            // Enter
            if (Status0 & MenuOn)
                Get_Menu(3);
        }
    }
}


bool Menu_Adapt(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("ADAP"));

    return false;
}

bool Menu_AdaptSub1(uint8_t task __attribute__((unused)))
{
    ValveTop = 0; // force adaptation
    StartMain();
    Status0 &= ~MenuOn;

    return false;
}

bool Menu_Dbg_1(uint8_t task __attribute__((unused)))
{
    char b[4];
    switch (FuzzyVal & 0xfc)
    {
        case FuzzOK:    b[0] = 'O'; break;
        case FuzzWarm:  b[0] = 'W'; break;
        case FuzzHot:   b[0] = 'H'; break;
        case FuzzCool:  b[0] = 'C'; break;
        case FuzzCold:  b[0] = 'D'; break;
        case FuzzAbove: b[0] = 'A'; break;
        case FuzzBelow: b[0] = 'B'; break;
    }
    b[1] = '-';
    switch (FuzzyVal & 0x03)
    {
        case TStatic:   b[2] = 'S'; break;
        case TRise:     b[2] = 'R'; break;
        case TFall:     b[2] = 'F'; break;
    }
    b[3] = 0;
    PutFormatted(FSTR("FUZZ\n%4s"), b);

    return false;
}

bool Menu_Dbg_2(uint8_t task __attribute__((unused)))
{
    PutFormatted(FSTR("POSI\n%4d"), Position);

    return false;
}

bool Menu_Dbg_3(uint8_t task __attribute__((unused)))
{
    PutFormatted(FSTR("VTOP\n%4d"), ValveTop);

    return false;
}

bool Menu_Dbg_4(uint8_t task __attribute__((unused)))
{
    ClearColon();
    PutFormatted(FSTR("RWAY\n%+4d"), RegWay);

    return false;
}

bool Menu_Dbg_5(uint8_t task __attribute__((unused)))
{
    SetColon(2);
    PutFormatted(FSTR(DELTA "TMP\n%02X%02X"), (unsigned)DeltaTemp1, (unsigned)DeltaTemp2);

    return false;
}

bool Menu_Dbg_FW(uint8_t task __attribute__((unused)))
{
    ClearColon();
    PutFormatted(FSTR("FIRM\nV%03d"), FW_Version);

    return false;
}

bool Menu_Debug(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("DBUG"));

    return false;
}

bool Menu_Fens(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("FENS"));

    return false;
}

bool Menu_FensSub1(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("HOCH"));

    return false;
}

bool Menu_FensSub11(uint8_t task __attribute__((unused)))
{
    PutFormatted(FSTR("%d0 M"), (MenuLow & 0x0F));

    return false;
}

bool Menu_FensSub2(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("MITT"));

    return false;
}

bool Menu_FensSub3(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("NIED"));

    return false;
}

bool Menu_Inst(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("INST"));

    return false;
}

bool Menu_InstSub1(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("<<<<"));

    return false;
}

bool Menu_Mode(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("MODE"));

    return false;
}

bool Menu_ModeSub1(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("MANU\n    "));

    return false;
}

bool Menu_ModeSub11(uint8_t task __attribute__((unused)))
{
    ClearSymbols();
    PutSymbol(LCD_Auto_CLR, 3);
    PutSymbol(LCD_Manu_SET, 3);
    OpMode = MANU;
    CurrAutoMode = OFF;
    Clear_Bargraph();
    MenuModeOff();

    return false;
}

bool Menu_ModeSub2(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("AUTO\n    "));

    return false;
}

bool Menu_ModeSub21(uint8_t task __attribute__((unused)))
{
    PutSymbol(LCD_Manu_CLR, 3);
    PutSymbol(LCD_Auto_SET, 3);
    OpMode = AUTO;
    Show_TimerSetBar(DailyTimer + TOD.WDays * 9, 0);
    MenuModeOff();

    return false;
}

bool Menu_Offs(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("OFFS"));

    return false;
}

bool Menu_OffsSub1(uint8_t task __attribute__((unused)))
{
    Status0 |= MenuWork;

    switch (task)
    {
    case 1:
        // Minus
        TempOffset -= 5;
        if (TempOffset < -50)
            TempOffset = -50;
        break;

    case 2:
        // Plus
        TempOffset += 5;
        if (TempOffset > 50)
            TempOffset = 50;
        break;

    case 3:
        // Enter
        Status0 &= ~MenuWork;
        ClearColon();
        MenuLow = 0;
        eeprom_write_word((uint16_t *)&eemem.temperatures.offset, (uint16_t)TempOffset);

        return true;
    }

    SetPoint();
    PutFormatted(FSTR("%+03d" DEGREE), TempOffset);

    return false;
}


bool Menu_Prog(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("PROG"));

    return false;
}

bool Menu_ProgSub1(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("TAG1\n    "));
    ClearWeekDays();
    PutWeekDay(LCD_Mo_SET, 1);

    return false;
}

static void CopyTimerBlock(uint8_t block)
{
    switch (block)
    {
        case 0x80:
            // _Copy1_5
            for (uint8_t i = 0; i < 5; i++)
            {
                memcpy(DailyTimer + i * TIMPERDAY, DailyTimer + TIMPERDAY * 7, TIMPERDAY);
                eeprom_write_block(DailyTimer + i * TIMPERDAY, eemem.dailytimer + i * TIMPERDAY, TIMPERDAY);
            }
            break;

        case 0x90:
            // _Copy1_6
            for (uint8_t i = 0; i < 6; i++)
            {
                memcpy(DailyTimer + i * TIMPERDAY, DailyTimer + TIMPERDAY * 8, TIMPERDAY);
                eeprom_write_block(DailyTimer + i * TIMPERDAY, eemem.dailytimer + i * TIMPERDAY, TIMPERDAY);
            }
            break;

        case 0xa0:
            // _Copy1_7
            for (uint8_t i = 0; i < 7; i++)
            {
                memcpy(DailyTimer + i * TIMPERDAY, DailyTimer + TIMPERDAY * 9, TIMPERDAY);
                eeprom_write_block(DailyTimer + i * TIMPERDAY, eemem.dailytimer + i * TIMPERDAY, TIMPERDAY);
            }
            break;

        case 0xb0:
            // 6-7
            for (uint8_t i = 5; i < 7; i++)
            {
                memcpy(DailyTimer + i * TIMPERDAY, DailyTimer + TIMPERDAY * 10, TIMPERDAY);
                eeprom_write_block(DailyTimer + i * TIMPERDAY, eemem.dailytimer + i * TIMPERDAY, TIMPERDAY);
            }
            break;

        default:
            return;
    }
}


static bool MenuProg_Com(uint8_t task)
{
    uint8_t menu_num = MenuLow - 0x11;
    uint8_t x = ((menu_num & 0xF0) >> 4) * TIMPERDAY /* number of timers per Day */;
    uint8_t *BarBase = DailyTimer + x;
    uint8_t *p = &BarBase[menu_num & 0x0F];
    if ((MenuLow & 1) == 0)
    {
        // second part of a day timer
        if (p[0] == 0 /* not set */ &&
            p[-1] != 0 /* start timer is set */)
            p[0] = p[-1];  // set end time to start time
    }
    uint8_t y = *p; // TempH in asm
    switch (task)
    {
        default:
            // MPS11
            break;

        case 1:
            // ProgSub11Minus
            y--;
            if (y == 254)
                y = 143;
            break;

        case 2:
            // ProgSub11Plus
            y++;
            if (y == 144)
                y = 255;
            break;

        case 3:
            // ProgSub11Enter
            ClearColon();
            Status0 &= ~MenuWork;
            // check for even submenu, 2nd part of a day timer
            if ((MenuLow & 1) != 0
                // if both part of a day timer are inactive...
                || p[-1] != 255
                || *p != 255)
            {
                // ProgSub11Enter1
                // check for last subsub menu (night timer)
                if ((MenuLow & 0x0F) == TIMPERDAY)
                {
                    // save state to EEPROM
                    eeprom_write_block(BarBase, eemem.dailytimer + x, TIMPERDAY);
                    if ((MenuLow & 0xF0) >= 0x80)
                        // if block menu, then copy block data to selected days
                        CopyTimerBlock(MenuLow & 0xF0);
                    ClearSymbols();
                    // IncSub[2]
                    MenuLow = (MenuLow & 0xF0) + 0x10;
                    return true;
                }
                // IncSubSub[3]
                MenuLow++;

                return true;
            }
            else
            {
                // ...set all day timers after one inactive Timer also to inactive state...
                uint8_t i = (MenuLow & 0x0F) - 8;
                // ClearInactiveTimers
                while (i-- != 0)
                    *p++ = 255;
                MenuLow = (MenuLow & 0xF0) + 8;
                // IncSubSub[2]
                MenuLow++;

                return true;
            }
            break;
    }
    *p = y;
    SetColon(1);
    if (y == 255)
    {
        // TimerInactive
        PutString(FSTR("----\n    "));
    }
    else
    {
        div_t d;
        d = div((int)y, 6);
        switch (MenuLow & 0x0F)
        {
            case 1:
            case 3:
            case 5:
            case 7:
                PutFormatted(FSTR("%2d%d0\nSTRT"), d.quot, d.rem);
                break;

            case 2:
            case 4:
            case 6:
            case 8:
                PutFormatted(FSTR("%2d%d0\nENDE"), d.quot, d.rem);
                break;

            default:
                PutFormatted(FSTR("%2d%d0\n    "), d.quot, d.rem);
                break;
        }
        Show_TimerSetBar(BarBase, &BarBase[menu_num & 0x0F]);
    }

    return false;
}


bool Menu_ProgSub11(uint8_t task)
{
    Status0 |= MenuWork;
    CopyWeekDays1_2();
    PutSymbol(LCD_InHouse_SET, 3);
    return MenuProg_Com(task);
}

bool Menu_ProgSub12(uint8_t task)
{
    Status0 |= MenuWork;
    ClearSymbols();
    PutSymbol(LCD_OffHouse_SET, 3);
    return MenuProg_Com(task);
}

bool Menu_ProgSub13(uint8_t task)
{
    Status0 |= MenuWork;
    ClearSymbols();
    PutSymbol(LCD_InHouse_SET, 3);
    return MenuProg_Com(task);
}

bool Menu_ProgSub14(uint8_t task)
{
    Status0 |= MenuWork;
    ClearSymbols();
    PutSymbol(LCD_Moon_SET, 3);
    return MenuProg_Com(task);
}

bool Menu_ProgSub2(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("TAG2\n    "));
    ClearWeekDays();
    PutWeekDay(LCD_Di_SET, 1);

    return false;
}

bool Menu_ProgSub3(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("TAG3\n    "));
    ClearWeekDays();
    PutWeekDay(LCD_Mi_SET, 1);

    return false;
}

bool Menu_ProgSub4(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("TAG4\n    "));
    ClearWeekDays();
    PutWeekDay(LCD_Do_SET, 1);

    return false;
}

bool Menu_ProgSub5(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("TAG5\n    "));
    ClearWeekDays();
    PutWeekDay(LCD_Fr_SET, 1);

    return false;
}

bool Menu_ProgSub6(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("TAG6\n    "));
    ClearWeekDays();
    PutWeekDay(LCD_Sa_SET, 1);

    return false;
}

bool Menu_ProgSub7(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("TAG7\n    "));
    ClearWeekDays();
    PutWeekDay(LCD_So_SET, 1);

    return false;
}

bool Menu_ProgSub8(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("T1:5\n    "));
    ClearWeekDays();
    PutWeekDay(LCD_Mo_SET, 1);
    PutWeekDay(LCD_Di_SET, 1);
    PutWeekDay(LCD_Mi_SET, 1);
    PutWeekDay(LCD_Do_SET, 1);
    PutWeekDay(LCD_Fr_SET, 1);

    return false;
}

bool Menu_ProgSub9(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("T1:6\n    "));
    ClearWeekDays();
    PutWeekDay(LCD_Mo_SET, 1);
    PutWeekDay(LCD_Di_SET, 1);
    PutWeekDay(LCD_Mi_SET, 1);
    PutWeekDay(LCD_Do_SET, 1);
    PutWeekDay(LCD_Fr_SET, 1);
    PutWeekDay(LCD_Sa_SET, 1);

    return false;
}

bool Menu_ProgSubA(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("T1:7\n    "));
    ClearWeekDays();
    PutWeekDay(LCD_Mo_SET, 1);
    PutWeekDay(LCD_Di_SET, 1);
    PutWeekDay(LCD_Mi_SET, 1);
    PutWeekDay(LCD_Do_SET, 1);
    PutWeekDay(LCD_Fr_SET, 1);
    PutWeekDay(LCD_Sa_SET, 1);
    PutWeekDay(LCD_So_SET, 1);

    return false;
}

bool Menu_ProgSubB(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("T6:7\n    "));
    ClearWeekDays();
    PutWeekDay(LCD_Sa_SET, 1);
    PutWeekDay(LCD_So_SET, 1);

    return false;
}

bool Menu_Reset(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("RES "));

    return false;
}

bool Menu_ResetSub1(uint8_t task __attribute__((unused)))
{
    PutString(FSTR(" OK "));

    return false;
}

bool Menu_ResetSub2(uint8_t task)
{
    Status0 |= MenuWork;

    switch (task)
    {
        case 3:
            // Enter
        {
            // Clear eeprom
            uint8_t *eeptr;
            size_t i;
            for (i = 0, eeptr = (uint8_t *)&eemem; i < sizeof(eemem); i++, eeptr++)
                eeprom_write_byte(eeptr, 0xFF);
            // Clear in-memory programming data
            ReadBack_Progdata();
            // Restart from scratch
            ValveTop = 0; // force adaptation
            StartMain();
            Status0 &= ~MenuOn;
            Status0 &= ~MenuWork;
            MenuLow = 0;

            return true;
        }

        default:
            // Everything else: cancelled
            PutString(FSTR("ABBR"));

            return false;
    }
}


bool Menu_Temp(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("TEMP"));

    return false;
}

bool Menu_TempSub1(uint8_t task)
{
    Status0 |= MenuWork;

    switch (task)
    {
    case 1:
        // InHouseMinus
        InHouseTemp -= 5;
        if (InHouseTemp < 40)
            InHouseTemp = 40;
        break;

    case 2:
        // InHousePlus
        InHouseTemp += 5;
        if (InHouseTemp > 350)
            InHouseTemp = 350;
        break;

    case 3:
        // InHouseEnter
        Status0 &= ~MenuWork;
        ClearPoint();
        PutSymbol(LCD_InHouse_CLR, 3);
        eeprom_write_word(&eemem.temperatures.inhouse, InHouseTemp);
        // IncSub
        MenuLow = (MenuLow & 0xF0) + 0x10;

        return true;
    }

    // MTS11
    PutFormatted(FSTR("%3d" DEGREE "\n    "), InHouseTemp);
    SetPoint();
    PutSymbol(LCD_InHouse_SET, 3);

    return false;
}

bool Menu_TempSub2(uint8_t task)
{
    Status0 |= MenuWork;

    switch (task)
    {
    case 1:
        // Minus
        OffHouseTemp -= 5;
        if (OffHouseTemp < 40)
            OffHouseTemp = 40;
        break;

    case 2:
        // Plus
        OffHouseTemp += 5;
        if (OffHouseTemp > 350)
            OffHouseTemp = 350;
        break;

    case 3:
        // Enter
        Status0 &= ~MenuWork;
        ClearPoint();
        PutSymbol(LCD_OffHouse_CLR, 3);
        eeprom_write_word(&eemem.temperatures.offhouse, OffHouseTemp);
        // IncSub
        MenuLow = (MenuLow & 0xF0) + 0x10;

        return true;
    }

    PutFormatted(FSTR("%3d" DEGREE "\n    "), OffHouseTemp);
    SetPoint();
    PutSymbol(LCD_OffHouse_SET, 3);

    return false;
}

bool Menu_TempSub3(uint8_t task)
{
    Status0 |= MenuWork;

    switch (task)
    {
    case 1:
        // Minus
        NightTemp -= 5;
        if (NightTemp < 40)
            NightTemp = 40;
        break;

    case 2:
        // Plus
        NightTemp += 5;
        if (NightTemp > 350)
            NightTemp = 350;
        break;

    case 3:
        // Enter
        Status0 &= ~MenuWork;
        MenuLow = 0;
        ClearPoint();
        eeprom_write_word(&eemem.temperatures.night, NightTemp);
        PutSymbol(LCD_Moon_CLR, 3);

        return true;
    }

    PutFormatted(FSTR("%3d" DEGREE "\n    "), NightTemp);
    SetPoint();
    PutSymbol(LCD_Moon_SET, 3);

    return false;
}

bool Menu_Urla(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("URLA"));

    return false;
}

bool Menu_UrlaSub1(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("DATE"));

    return false;
}

bool Menu_UrlaSub2(uint8_t task)
{
    Status0 |= MenuWork;
    switch (task)
    {
    case 1:
        // YearsMinus
        if (Urlaub.Years == 0)
            Urlaub.Years = 99;
        else
            Urlaub.Years--;
        break;

    case 2:
        // YearsPlus
        if (Urlaub.Years == 99)
            Urlaub.Years = 0;
        else
            Urlaub.Years++;
        break;

    case 3:
        // YearsEnter
        Status0 &= ~MenuWork;

        if (Urlaub.Years == 0)
        {
            // vacation function turned off
            ClearPoint();
            uint8_t toddow = CalcDayOfWeek(&TOD);
            ClearWeekDays();
            PutWeekDay(toddow | 0x80, 3);
            MenuLow = 0;
        }
        else
        {
            // vacation function enabled, proceed to next submenu
            MenuLow = (MenuLow & 0xF0) + 0x10;
        }

        return true;
    }

    if (Urlaub.Years == 0)
    {
        PutString(FSTR("URLA\nOFF "));
    }
    else
    {
        PutFormatted(FSTR("20%02d\nURLA"), Urlaub.Years);

        uint8_t dow = CalcDayOfWeek(&Urlaub);
        ClearWeekDays();
        PutWeekDay(dow | 0x80, 3);
    }

    return false;
}

// set Months value
bool Menu_UrlaSub3(uint8_t task)
{
    Status0 |= MenuWork;
    switch (task)
    {
    case 1:
        // MonthsMinus
        if (Urlaub.Months == 0)
            Urlaub.Months = 11;
        else
            Urlaub.Months--;
        break;

    case 2:
        // MonthsPlus
        if (Urlaub.Months == 11)
            Urlaub.Months = 0;
        else
            Urlaub.Months++;
        break;

    case 3:
        // MonthsEnter
        Status0 &= ~MenuWork;
        // IncSub[4]
        MenuLow = (MenuLow & 0xF0) + 0x10;

        return true;
    }
    PutFormatted(FSTR("%02d%02d\n%02dLA"), Urlaub.Days + 1, Urlaub.Months + 1, Urlaub.Days + 1);
    SetPoint();

    uint8_t dow = CalcDayOfWeek(&Urlaub);
    ClearWeekDays();
    PutWeekDay(dow | 0x80, 3);

    return false;
}

// set Days value
bool Menu_UrlaSub4(uint8_t task)
{
    Status0 |= MenuWork;
    switch (task)
    {
    case 1:
        // DaysMinus
        if (Urlaub.Days == 0)
            Urlaub.Days = MonthLastDay(&Urlaub);
        else
            Urlaub.Days--;
        break;

    case 2:
        // DaysPlus
        if (++Urlaub.Days == MonthLastDay(&Urlaub))
            Urlaub.Days = 0;
        break;

    case 3:
        // DaysEnter
        Status0 &= ~MenuWork;
        ClearPoint();
        uint8_t toddow = CalcDayOfWeek(&TOD);
        ClearWeekDays();
        PutWeekDay(toddow | 0x80, 3);
        MenuLow = 0;

        return true;
    }
    PutFormatted(FSTR("%02d%02d\nUR%02d"), Urlaub.Days + 1, Urlaub.Months + 1, Urlaub.Months + 1);

    uint8_t dow = CalcDayOfWeek(&Urlaub);
    ClearWeekDays();
    PutWeekDay(dow | 0x80, 3);

    return false;
}

bool Menu_Zeit(uint8_t task __attribute__((unused)))
{
    PutString(FSTR("ZEIT"));

    return false;
}

// set Year value
bool Menu_ZeitSub1(uint8_t task)
{
    Status0 |= MenuWork;
    switch (task)
    {
    case 1:
        // YearsMinus
        if (TOD.Years == 0)
            TOD.Years = 99;
        else
            TOD.Years--;
        break;

    case 2:
        // YearsPlus
        if (TOD.Years == 99)
            TOD.Years = 0;
        else
            TOD.Years++;
        break;

    case 3:
        // YearsEnter
        Status0 &= ~MenuWork;
        // IncSub[3]
        MenuLow = (MenuLow & 0xF0) + 0x10;

        return true;
    }
    PutFormatted(FSTR("20%02d\n    "), TOD.Years);

    uint8_t dow = CalcDayOfWeek(&TOD);
    ClearWeekDays();
    PutWeekDay(dow | 0x80, 3);

    return false;
}

// set Months value
bool Menu_ZeitSub2(uint8_t task)
{
    Status0 |= MenuWork;
    switch (task)
    {
    case 1:
        // MonthsMinus
        if (TOD.Months == 0)
            TOD.Months = 11;
        else
            TOD.Months--;
        break;

    case 2:
        // MonthsPlus
        if (TOD.Months == 11)
            TOD.Months = 0;
        else
            TOD.Months++;
        break;

    case 3:
        // MonthsEnter
        Status0 &= ~MenuWork;
        // IncSub[4]
        MenuLow = (MenuLow & 0xF0) + 0x10;

        return true;
    }
    PutFormatted(FSTR("%02d%02d\n%02d  "), TOD.Days + 1, TOD.Months + 1, TOD.Days + 1);
    SetPoint();

    uint8_t dow = CalcDayOfWeek(&TOD);
    ClearWeekDays();
    PutWeekDay(dow | 0x80, 3);

    return false;
}

// set Days value
bool Menu_ZeitSub3(uint8_t task)
{
    Status0 |= MenuWork;
    switch (task)
    {
    case 1:
        // DaysMinus
        if (TOD.Days == 0)
            TOD.Days = MonthLastDay(&TOD);
        else
            TOD.Days--;
        break;

    case 2:
        // DaysPlus
        if (++TOD.Days == MonthLastDay(&TOD))
            TOD.Days = 0;
        break;

    case 3:
        // DaysEnter
        Status0 &= ~MenuWork;
        ClearPoint();
        // IncSub[5]
        MenuLow = (MenuLow & 0xF0) + 0x10;

        return true;
    }
    PutFormatted(FSTR("%02d%02d\n  %02d"), TOD.Days + 1, TOD.Months + 1, TOD.Months + 1);

    uint8_t dow = CalcDayOfWeek(&TOD);
    ClearWeekDays();
    PutWeekDay(dow | 0x80, 3);

    return false;
}

// set Hours value
bool Menu_ZeitSub4(uint8_t task)
{
    Status0 |= MenuWork;
    switch (task)
    {
    case 1:
        // HoursMinus
        if (TOD.Hours == 0)
            TOD.Hours = 23;
        else
            TOD.Hours--;
        break;

    case 2:
        // HoursPlus
        if (TOD.Hours == 23)
            TOD.Hours = 0;
        else
            TOD.Hours++;
        break;

    case 3:
        // HoursEnter
        Status0 &= ~MenuWork;
        // IncSub[6]
        MenuLow = (MenuLow & 0xF0) + 0x10;

        return true;
    }
    PutFormatted(FSTR("%2d%02d\n  %02d"), TOD.Hours, TOD.Minutes, TOD.Minutes);
    SetColon(3);

    return false;
}

// set Minutes value
bool Menu_ZeitSub5(uint8_t task)
{
    Status0 |= MenuWork;
    switch (task)
    {
    case 1:
        // MinutesMinus
        if (TOD.Minutes == 0)
            TOD.Minutes = 59;
        else
            TOD.Minutes--;
        break;

    case 2:
        // MinutesPlus
        if (TOD.Minutes == 59)
            TOD.Minutes = 0;
        else
            TOD.Minutes++;
        break;

    case 3:
        // MinutesEnter
        Status0 &= ~MenuWork;
        ClearColon();
        MenuLow = 0;

        return true;
    }
    PutFormatted(FSTR("%2d%02d\n%2d  "), TOD.Hours, TOD.Minutes, TOD.Hours);
    SetColon(3);

    return false;
}

/* --- From ADC.inc ---------------------------------------------- */

void Measure_Motor_Current(void)
{
    if (!(Status0 & Adapt))
        return;
    ADMUX = 0b01000010; // AVcc as reference, channel 2, motor current
    _delay_us(0.3);
    ADCSRA = 0b11000100; // ADC enable, start ADC conversion, ADC Prescaler 16
    while (ADCSRA & _BV(ADSC)) {}
    ADCSRA = 0b00000100; // ADC disable, ADC Prescaler 16
    MotorCurrent = 0x3FF - ADC;
}

/* --- From RTC.inc ---------------------------------------------- */

/* RealTimeClock timer, automatic DST switch */
void Clock(void)
{
    // execute only ,if prescaler1 is 0
    if (!(Status1 & SecondTick))
        return;
    Status1 &= ~SecondTick;
    if (++TOD.Seconds != 60)
        return;
    TOD.Seconds = 0;
    if (--TimeAdjust == 0)
    {
        PSC1 = 5; // adjust value for prescaler1 interval (default 4)
        TimeAdjust = 35; // TimeAdjust interval counter
    }
    TOD.Minutes++;
    if (TOD.Minutes == 60)
    {
        TOD.Minutes = 0;
        TOD.Hours++;
        // if DST toggle has already done, skip DST check
        if (!(Status1 & DST_OnOff))
        {
            if (TOD.Days >= 24 && // last week on month?
                TOD.WDays == 6) // last sunday on month?
                switch (TOD.Months)
                {
                    // October?
                    case 9:
                        // Summer_Winter
                        if (TOD.Hours == 3)
                        {
                            TOD.Hours--;
                            Status1 |= DST_OnOff;
                        }
                        break;

                        // March?
                    case 2:
                        // Winter_Summer
                        if (TOD.Hours == 2)
                        {
                            TOD.Hours++;
                            Status1 |= DST_OnOff;
                        }
                        break;
                }
        }
        if (TOD.Hours == 24)
        {
            TOD.Hours = 0;
            Status1 &= ~DST_OnOff;
            TOD.Days++;
            if (++TOD.WDays == 7)
                TOD.WDays = 0;
            if (OpMode == AUTO)
                Show_TimerSetBar(DailyTimer + TOD.WDays * 9, 0);
            ClearWeekDays();
            PutWeekDay(TOD.WDays | 0x80, 3);
            if (TOD.Days == MonthLastDay(&TOD))
            {
                TOD.Days = 0;
                if (++TOD.Months == 12)
                {
                    TOD.Months = 0;
                    TOD.Years++;
                }
            }
        }
    }

    if (Urlaub.Years && Urlaub.Months && Urlaub.Days)
    {
        // vacation mode active, see whether it needs to be turned off
        if (Urlaub.Years == TOD.Years &&
            Urlaub.Months == TOD.Months &&
            Urlaub.Days == TOD.Days)
        {
            Urlaub.Years = 0;
            Urlaub.Months = 0;
            Urlaub.Days = 0;
            PutSymbol(LCD_Case_CLR, 3);
        }
    }

    if (OpMode == AUTO && TOD.Minutes % 10 == 0)
        // new ten minute interval, see whether we have to switch modes
        EvalAutoMode();
}

// w=(d+MonthGauss+y+(y/4)+(c/4)+5*c) mod7 // Sunday=0...Saturday=6
// d=1..31 // y=Year ..nn // c= Year nn..
// Jan. + Feb. -> y=y-1
uint8_t CalcDayOfWeek(struct time *clock)
{
    static const __flash uint8_t MonthGauss[] =
        {
            28,31,2,5,7,10,12,15,18,20,23,25
        };
    uint8_t mon = MonthGauss[clock->Months];
    uint8_t d = clock->Days + 1;
    uint8_t y = clock->Years;
    // if months are 0 or 1 (January or February), decrement y
    if (clock->Months < 2)
        y--;
    d += mon; // add GaussMonths
    d += y; // add Year
    const uint8_t c = 20; // century
    d += y / 4;
    d += c / 4;
    d += 5 * c;
    while (d > 7)
        d -= 7;
    if (d == 0)
        d = 6;
    else
        d -= 1;
    clock->WDays = d;

    return clock->WDays;
}

// look for last Day of a month
uint8_t MonthLastDay(struct time *clock)
{
    static const __flash uint8_t MonthDayNo[] =
        {
            31,28,31,30,31,30,31,31,30,31,30,31
        };
    uint8_t d = MonthDayNo[clock->Months];
    if (clock->Years % 4 == 0 && // check for leap year
        clock->Months == 1) // if month is february, add 1 day
        d++;
    return d;
}

/* --- From ExternalPeripheral.inc ------------------------------- */

// Soft SPI, uses MOSI, MISO and SCK // 1 Data transmit on one call
uint8_t Soft_SPI(uint8_t inp)
{
    uint8_t outp = 0;

    for (uint8_t i = 0; i < 8; i++, inp >>= 1, outp <<= 1)
    {
        if (inp & 1)
            SSPI_PORT |= SSPI_MOSI;
        else
            SSPI_PORT &= ~SSPI_MOSI;
        _delay_us(0.25);
        SSPI_PORT |= SSPI_SCK;
        _delay_us(0.25);
        if (SSPI_PIN & SSPI_MISO)
            outp |= 1;
        SSPI_PORT &= ~SSPI_SCK;
    }
    return outp;
}
