
;--------------------------------------
; get pin values from button pins, process debouncing and rotary decoding
ReadButtons:
 in		Temp, PinB
 com	Temp
 andi	Temp, 0b11110001			;mask used pins and swap nibbles, put all button pins together -> 000bbbbb
 swap	Temp
 mov	TempH, Temp					;copy pin values to process buttons independently
 andi	TempH, 0b0000111			;mask buttons
 cpi	TempH, 0					;if no button is pressed, clear button value and status flag
 brne	_RButt1
 sts	Buttons, TempH
 cbi	Status0, _NewButton
 rjmp	_RButt2

_RButt1:
 sts	Buttons, TempH
 sbi	Status0, _NewButton
 
_RButt2: 
 andi	Temp, 0b00011000			;mask rotary pins
 lsr	Temp
 lsr	Temp
 lsr	Temp
 lds	TempH, RotaryTemp			
 or		TempH, Temp
 lsl	TempH
 lsl	TempH
 andi	TempH, 0b00111100			;shift and store current rotary pin state
 sts	RotaryTemp, TempH
 cpi	TempH, 0b00000100			;check for first half step in clockwise direction
 breq	_ROTStepCW1
 cpi	TempH, 0b00111000			;check for first half step in clockwise direction
 breq	_ROTStepCW1
 cpi	TempH, 0b00001000			;check for first half step in counter-clockwise direction
 breq	_ROTStepCCW1
 cpi	TempH, 0b00110100			;check for first half step in counter-clockwise direction
 breq	_ROTStepCCW1
 
 lds	Temp, Rotary				;if first half step was detected, check for second half step CW
 andi	Temp, 0b11000000
 cpi	Temp, 0b10000000
 breq	_RotStepCW2
 cpi	Temp, 0b01000000			;if first half step was detected, check for second half step CCW
 breq	_RotStepCCW2
 ret

_ROTStepCW1:
 lds	Temp, Rotary
 andi	Temp, 0b00111111
 ori	Temp, 0b10000000			;mark first half step detected CW
 sts	Rotary, Temp
 ret
 
_ROTStepCCW1:
 lds	Temp, Rotary
 andi	Temp, 0b00111111
 ori	Temp, 0b01000000			;mark first half step detected CCW
 sts	Rotary, Temp
 ret

_ROTStepCW2:
 cpi	TempH, 0b00011100			;check for second half step CW
 breq	_ROTStepCWP
 cpi	TempH, 0b00100000
 breq	_ROTStepCWP
 ret

_ROTStepCWP:
 ldi	Temp, 0b00000011			;set CW flag and new rotary flag
 sts	Rotary, Temp
 ;sbi	DebugPort, Debug1
 ret

_ROTStepCCW2:
 cpi	TempH, 0b00101100			;check for second half step CCW
 breq	_ROTStepCCWP
 cpi	TempH, 0b00010000
 breq	_ROTStepCCWP
 ret

_ROTStepCCWP:
 ldi	Temp, 0b00000010			;set new rotary flag
 sts	Rotary, Temp
 ;sbi	DebugPort, Debug2
 ret



 
;--------------------------------------------------
User_Action:
 lds	Temp, Rotary
 sbrc	Temp, 1					;if new rotary value or new button press detected, execute
 rjmp	_UAC_Rotary

 sbis	Status0, _NewButton
 ret
  
_UAC_Button:
 cbi	Status0, _NewButton
 lds	Temp, Buttons
 cpi	Temp, _Menu_Button
 breq	_Toggle_Menu
 cpi	Temp, _OK_Button
 breq	_Enter
 ret
 

_Toggle_Menu:
 sbic	Status0, _MenuOn
 rjmp	_MenuModeOff
 
_MenuModeOn:
 sbi	Status0, _MenuOn
 sts	MenuHigh, Null
 sts	MenuLow, Null
 rcall	ClearColon
 rcall	ClearPoint 
 ldi	Temp, 0
 rcall	Get_Menu
 ret

_MenuModeOff:
 cbi	Status0, _MenuOn
 cbi	Status0, _MenuWork
 ldi	Temp, 1
 sts	UserDisplay, Temp
 ldi	Temp, 3
 sts	DispTimer, Temp
 lds	Temp, WDays								;restore Week day display after Menu setting
 ori	Temp, $80
 rcall	ClearWeekDays
 ldi	TempH, $30
 rcall	PutWeekDay
 rcall	ClearSymbols
 ret

_Enter:
 sbis	Status0, _MenuOn
 ret
 ldi	Temp, 3
 rcall	Get_Menu
 ret


_UAC_Rotary:
 cbr	Temp, (1<<1)							;clear new rotary flag
 sts	Rotary, Temp
 sbrc	Temp, 0
 rjmp	_RotaryPlus

_RotaryMinus:
 sbic	Status0, _MenuOn
 rjmp	_RM1
 rcall	Set_SetTemperature_Down
 lds	Temp, TempIntL						;NTC temperature  = OldTemperature
 lds	TempH, TempIntH
 sts	TempIntOldL, Temp
 sts	TempIntOldH, TempH
 ret
 
_RM1:
 ldi	Temp, 1
 rcall	Get_Menu
 ret
	
_RotaryPlus:
 sbic	Status0, _MenuOn
 rjmp	_RP1
 rcall	Set_SetTemperature_Up
 lds	Temp, TempIntL						;NTC temperature  = OldTemperature
 lds	TempH, TempIntH
 sts	TempIntOldL, Temp
 sts	TempIntOldH, TempH
 ret

_RP1:
 ldi	Temp, 2
 rcall	Get_Menu
 ret


;------------------------------------------------------------------
Set_SetTemperature_Up:
 lds	Temp, SetTempL
 lds	TempH, SetTempH
 
 ldi	Temp3, high(300)
 cpi	Temp, low(300)
 cpc	TempH, Temp3
 breq	_SSTU1
 
 ldi	Temp3, 5
 add	Temp, Temp3
 adc	TempH, Null
 sts	SetTempL, Temp
 sts	SetTempH, TempH
 sts	UserDisplay, Temp3 
 
_SSTU1:
 ldi	Temp, 10
 sts	ADCPrescaler, Temp
 ret

;-----------------------------------------
Set_SetTemperature_Down:
 lds	Temp, SetTempL
 lds	TempH, SetTempH
 
 ldi	Temp3, high(40)
 cpi	Temp, low(40)
 cpc	TempH, Temp3
 breq	_SSTU1
 
 subi	Temp, 5
 sbc	TempH, Null
 sts	SetTempL, Temp
 sts	SetTempH, TempH
 ldi	Temp3, 5
 sts	UserDisplay, Temp3 
 rjmp	_SSTU1


;-----------------------------------------
 Get_Menu:
 ldi	ActiveCT, 20
 ;sbi	BacklightPort, Backlight_On

 sbic	Status0, _MenuWork					;if work menu is selected, skip new menu selection and go to current menu		
 rjmp	_MenuFound


_GM_00:
 cpi	Temp, 0
 breq	_GM_0
 cpi	Temp, 1
 breq	_GM_1x
 cpi	Temp, 2
 breq	_GM_2x
 cpi	Temp, 3
 breq	_GM_3x
 ret

_GM_1x:
 rjmp	_GM_1
  
_GM_2x:
 rjmp	_GM_2

_GM_3x:
 rjmp	_GM_3

_GM_0:
 clr	MenuID
 ldi	ZL, low (MenuTable*2)
 ldi	ZH, high (MenuTable*2)
 lds	Temp4, MenuHigh
 lds	Temp3, MenuLow
 
_GM_01:
 lpm	TempH, Z+
 lpm	Temp, Z+
 cp		Temp, Temp3  
 cpc	TempH, Temp4
 brne	_GM_02
 clr	Temp								;clear task value
 rjmp	_MenuFound
 
_GM_02:
 cpi	TempH, $FF
 breq	_MenuEnd
 inc	MenuID
 rjmp	_GM_01

_MenuEnd:
 lds	Temp, MenuLow
 cpi	Temp, 0	
 breq	_ME_Main

 andi	Temp, $0F							;if lower nibble is not 0, go to 2nd sub menu
 brne	_ME_Sub2
  
_ME_Sub1:
 ldi	TempH, $10
 lds	Temp, MenuLow
 cpi	Temp, $10							;if submenu is 1 and not found, exit
 breq	_ME_Sub1_Exit
 sbis	Status0, _MenuUpDn					;if search direction is up, add submenu and search
 rjmp	_ME_Sub1_Dn							;if search direction is down, sub submenu and search

 add	Temp, TempH
 brne	_ME_Sub11
 add	Temp, TempH
 rjmp	_ME_Sub11
 
_ME_Sub1_Dn:
 sub	Temp, TempH
 brne	_ME_Sub11

_ME_Sub1_Exit:
 sts	MenuLow, Null
 ret										;if subtracting down to 0 and no menu was found, exit

_ME_Sub11:
 sts	MenuLow, Temp
 rjmp	_GM_0
 
_ME_Sub2:
 lds	Temp, MenuLow
 andi	Temp, $0F
 cpi	Temp, $01							;if subsubmenu is 1 and not found, exit
 breq	_ME_Sub2_Exit
 sbis	Status0, _MenuUpDn					;if search direction is up, add submenu and search
 rjmp	_ME_Sub2_Dn							;if search direction is down, sub submenu and search

 inc	Temp 
 andi	Temp, $0F
 brne	_ME_Sub21
 inc	Temp
 rjmp	_ME_Sub21
 
_ME_Sub2_Dn:
 dec	Temp 
 brne	_ME_Sub21

_ME_Sub2_Exit:
 sts	MenuLow, Null
 ret										;if subtracted down to 0 and no menu was found, exit

_ME_Sub21:
 lds	TempH, MenuLow
 andi	TempH, $F0
 andi	Temp, $0F
 or		TempH, Temp
 sts	MenuLow, TempH
 rjmp	_GM_0
 

_ME_Main:
 lds	Temp, MenuHigh
 cpi	Temp, $FF
 breq	_ME_MainTop
 sts	MenuHigh, Null
 sts	MenuLow, Null
 rjmp	_GM_0

_ME_MainTop:
 ldi	Temp, $0A							;number of current main menues
 sts	MenuHigh, Temp
 sts	MenuLow, Null
 rjmp	_GM_0


_GM_1:
 cbi	Status0, _MenuUpDn
 lds	Temp, MenuLow
 andi	Temp, $F0
 breq	_DecMainMenu
 lds	Temp, MenuLow
 andi	Temp, $0F
 breq	_DecSublayer1

_DecSublayer2:
 ;cbi	DebugPort, Debug2
 lds	Temp, MenuLow						;decrement Menu SubLayer2
 dec	Temp 
 andi	Temp, $0F
 brne	_DSL2
 dec	Temp 

_DSL2: 
 lds	TempH, MenuLow
 andi	TempH, $F0
 andi	Temp, $0F
 or		TempH, Temp
 sts	MenuLow, TempH
 rjmp	_GM_0

_DecSublayer1:
 lds	Temp, MenuLow						;decrement Menu SubLayer1
 ldi	TempH, $10
 sub	Temp, TempH
 brne	_DSL1
 sub	Temp, TempH

_DSL1: 
 sts	MenuLow, Temp
 rjmp	_GM_0

_DecMainMenu:
 lds	Temp, MenuHigh
 dec	Temp
 sts	MenuHigh, Temp
 rjmp	_GM_0

_GM_2:
 sbi	Status0, _MenuUpDn
 lds	Temp, MenuLow
 andi	Temp, $F0
 breq	_IncMainMenu
 lds	Temp, MenuLow
 andi	Temp, $0F
 breq	_IncSublayer1

_IncSublayer2:
 ;sbi	DebugPort, Debug2
 lds	Temp, MenuLow						;increment Menu SubLayer2
 inc	Temp 
 andi	Temp, $0F
 brne	_DSL2
 inc	Temp 
 rjmp	_DSL2
 
_IncSublayer1:
 lds	Temp, MenuLow						;increment Menu SubLayer1
 ldi	TempH, $10
 add	Temp, TempH
 brne	_ISL1
 add	Temp, TempH

_ISL1: 
 sts	MenuLow, Temp
 rjmp	_GM_0

_IncMainMenu:
 lds	Temp, MenuHigh
 inc	Temp
 sts	MenuHigh, Temp
 rjmp	_GM_0


_GM_3:
 lds	Temp, MenuLow
 andi	Temp, $F0
 breq	_IncSub
 lds	Temp, MenuLow
 andi	Temp, $0F
 breq	_IncSubSub
 ret

_IncSub:
 lds	Temp, MenuLow						;switch to SubLayer1
 subi	Temp, -$10
 sts	MenuLow, Temp
 rjmp	_GM_0

_IncSubSub:
 lds	Temp, MenuLow						;switch to SubLayer2
 subi	Temp, -$01
 sts	MenuLow, Temp
 rjmp	_GM_0


;----------------------------------------------
_MenuFound:
 ldi	ZL, low (MenuJump)
 ldi	ZH, high (MenuJump)
 add	ZL, MenuID
 adc	ZH, Null
 ijmp

MenuJump:
 rjmp	Menu_Mode
 rjmp	Menu_ModeSub1
 rjmp	Menu_ModeSub11
 rjmp	Menu_ModeSub2
 rjmp	Menu_ModeSub21
 rjmp	Menu_Prog
 rjmp	Menu_ProgSub1
 rjmp	Menu_ProgSub11
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub14
 rjmp	Menu_ProgSub2
 rjmp	Menu_ProgSub11
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub14
 rjmp	Menu_ProgSub3
 rjmp	Menu_ProgSub11
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub14
 rjmp	Menu_ProgSub4
 rjmp	Menu_ProgSub11
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub14
 rjmp	Menu_ProgSub5
 rjmp	Menu_ProgSub11
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub14
 rjmp	Menu_ProgSub6
 rjmp	Menu_ProgSub11
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub14
 rjmp	Menu_ProgSub7
 rjmp	Menu_ProgSub11
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub14
 rjmp	Menu_ProgSub8
 rjmp	Menu_ProgSub11
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub14
 rjmp	Menu_ProgSub9
 rjmp	Menu_ProgSub11
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub14
 rjmp	Menu_ProgSubA
 rjmp	Menu_ProgSub11
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub13
 rjmp	Menu_ProgSub12
 rjmp	Menu_ProgSub14
 rjmp	Menu_Temp 
 rjmp	Menu_TempSub1
 rjmp	Menu_TempSub1
 rjmp	Menu_Zeit 
 rjmp	Menu_ZeitSub1
 rjmp	Menu_ZeitSub2
 rjmp	Menu_ZeitSub3
 rjmp	Menu_ZeitSub4
 rjmp	Menu_ZeitSub5
 rjmp	Menu_Fens
 rjmp	Menu_FensSub1
 rjmp	Menu_FensSub11
 rjmp	Menu_FensSub2
 rjmp	Menu_FensSub11
 rjmp	Menu_FensSub3
 rjmp	Menu_FensSub11
 rjmp	Menu_Reset
 rjmp	Menu_ResetSub1
 rjmp	Menu_Adapt
 rjmp	Menu_AdaptSub1
 rjmp	Menu_Urla
 rjmp	Menu_UrlaSub1
 rjmp	Menu_UrlaSub1
 rjmp	Menu_UrlaSub1
 rjmp	Menu_Inst
 rjmp	Menu_InstSub1
 rjmp	Menu_Offs
 rjmp	Menu_OffsSub1
 rjmp	Menu_Debug
 rjmp	Menu_Dbg_FW
 rjmp	Menu_Dbg_1
 rjmp	Menu_Dbg_2
 rjmp	Menu_Dbg_3
 rjmp	Menu_Dbg_4

;--------------------------------------------------------------
Menu_Mode:
 ldi	Temp, 'M'
 ldi	TempH, $30
 rcall	PutCharacter
 ldi	Temp, 'O'
 rcall	PutCharacter
 ldi	Temp, 'D'
 rcall	PutCharacter
 ldi	Temp, 'E'
 rcall	PutCharacter
 ret

Menu_ModeSub1:
 ldi	Temp, 'M'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, 'A'
 rcall	PutCharacter
 ldi	Temp, 'N'
 rcall	PutCharacter
 ldi	Temp, 'U'
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
 ret

Menu_ModeSub11:
 ldi	Temp, LCD_Auto_CLR
 ldi	TempH, $30
 rcall	PutSymbol
 ldi	Temp, LCD_Manu_SET
 ldi	TempH, $30
 rcall	PutSymbol
 rjmp	_MenuModeOff


Menu_ModeSub2:
 ldi	Temp, 'A'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, 'U'
 rcall	PutCharacter
 ldi	Temp, 'T'
 rcall	PutCharacter
 ldi	Temp, 'O'
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
 ret
 
Menu_ModeSub21:
 ldi	Temp, LCD_Manu_CLR
 ldi	TempH, $30
 rcall	PutSymbol
 ldi	Temp, LCD_Auto_SET
 ldi	TempH, $30
 rcall	PutSymbol
 rjmp	_MenuModeOff



;-----------------------------------------------------
Menu_Prog:
 ldi	Temp, 'P'
 ldi	TempH, $30
 rcall	PutCharacter
 ldi	Temp, 'R'
 rcall	PutCharacter
 ldi	Temp, 'O'
 rcall	PutCharacter
 ldi	Temp, 'G'
 rcall	PutCharacter
 ret

Menu_ProgSub1:
 ldi	Temp, 'T'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, 'A'
 rcall	PutCharacter
 ldi	Temp, 'G'
 rcall	PutCharacter
 ldi	Temp, '1'
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
 ldi	Temp, LCD_Mo_SET
 rcall	ClearWeekDays
 ldi	TempH, $10
 rcall	PutWeekDay
 ret

Menu_ProgSub2:
 ldi	Temp, 'T'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, 'A'
 rcall	PutCharacter
 ldi	Temp, 'G'
 rcall	PutCharacter
 ldi	Temp, '2'
 rcall	PutCharacter
 ldi	Temp, LCD_Di_SET
 rcall	ClearWeekDays
 ldi	TempH, $10
 rcall	PutWeekDay
 ret

Menu_ProgSub3:
 ldi	Temp, 'T'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, 'A'
 rcall	PutCharacter
 ldi	Temp, 'G'
 rcall	PutCharacter
 ldi	Temp, '3'
 rcall	PutCharacter
 ldi	Temp, LCD_Mi_SET
 rcall	ClearWeekDays
 ldi	TempH, $10
 rcall	PutWeekDay
 ret

Menu_ProgSub4:
 ldi	Temp, 'T'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, 'A'
 rcall	PutCharacter
 ldi	Temp, 'G'
 rcall	PutCharacter
 ldi	Temp, '4'
 rcall	PutCharacter
 ldi	Temp, LCD_Do_SET
 rcall	ClearWeekDays
 ldi	TempH, $10
 rcall	PutWeekDay
 ret

Menu_ProgSub5:
 ldi	Temp, 'T'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, 'A'
 rcall	PutCharacter
 ldi	Temp, 'G'
 rcall	PutCharacter
 ldi	Temp, '5'
 rcall	PutCharacter
 ldi	Temp, LCD_Fr_SET
 rcall	ClearWeekDays
 ldi	TempH, $10
 rcall	PutWeekDay
 ret

Menu_ProgSub6:
 ldi	Temp, 'T'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, 'A'
 rcall	PutCharacter
 ldi	Temp, 'G'
 rcall	PutCharacter
 ldi	Temp, '6'
 rcall	PutCharacter
 ldi	Temp, LCD_Sa_SET
 rcall	ClearWeekDays
 ldi	TempH, $10
 rcall	PutWeekDay
 ret

Menu_ProgSub7:
 ldi	Temp, 'T'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, 'A'
 rcall	PutCharacter
 ldi	Temp, 'G'
 rcall	PutCharacter
 ldi	Temp, '7'
 rcall	PutCharacter
 ldi	Temp, LCD_So_SET
 rcall	ClearWeekDays
 ldi	TempH, $10
 rcall	PutWeekDay
 ret

Menu_ProgSub8:
 ldi	Temp, 'T'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, '1'
 rcall	PutCharacter
 ldi	Temp, ':'
 rcall	PutCharacter
 ldi	Temp, '5'
 rcall	PutCharacter
 ldi	Temp, LCD_Mo_SET
 rcall	ClearWeekDays
 ldi	TempH, $10
 rcall	PutWeekDay
 ldi	Temp, LCD_Di_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_Mi_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_Do_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_Fr_SET
 rcall	PutWeekDay
 ret

Menu_ProgSub9:
 ldi	Temp, 'T'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, '1'
 rcall	PutCharacter
 ldi	Temp, ':'
 rcall	PutCharacter
 ldi	Temp, '6'
 rcall	PutCharacter
 ldi	Temp, LCD_Mo_SET
 rcall	ClearWeekDays
 ldi	TempH, $10
 rcall	PutWeekDay
 ldi	Temp, LCD_Di_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_Mi_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_Do_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_Fr_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_Sa_SET
 rcall	PutWeekDay
 ret

Menu_ProgSubA:
 ldi	Temp, 'T'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, '1'
 rcall	PutCharacter
 ldi	Temp, ':'
 rcall	PutCharacter
 ldi	Temp, '7'
 rcall	PutCharacter
 ldi	Temp, LCD_Mo_SET
 rcall	ClearWeekDays
 ldi	TempH, $10
 rcall	PutWeekDay
 ldi	Temp, LCD_Di_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_Mi_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_Do_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_Fr_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_Sa_SET
 rcall	PutWeekDay
 ldi	Temp, LCD_So_SET
 rcall	PutWeekDay
 ret

;....................................
Menu_ProgSub11:
 sbi	Status0, _MenuWork 
 push	Temp

 rcall	CopyWeekDays1_2

 ldi	Temp, LCD_InHouse_SET
 ldi	TempH, $30
 rcall	PutSymbol

_MenuProg_Com:
 rcall	SetColon
 
 ldi	XL, low(DailyTimer)
 ldi	XH, high(DailyTimer)
 lds	Temp, MenuLow
 subi	Temp, $11
 mov	TempH, Temp
 andi	Temp, $0F
 andi	TempH, $F0
 swap	TempH
 ldi	Temp3, 9								;number of timers per Day
 mul	TempH, Temp3
 add	XL, r0
 adc	XH, r1
 sts	BarBaseL, XL
 sts	BarBaseH, XH
 add	XL, Temp
 adc	XH, Null

 ld		TempH, X
 
 pop	Temp
 cpi	Temp, 1
 breq	_ProgSub11Minus
 cpi	Temp, 2
 breq	_ProgSub11Plus
 cpi	Temp, 3
 breq	_ProgSub11EnterX
 rjmp	_MPS11
 
_ProgSub11EnterX:
 rjmp	_ProgSub11Enter
 
_ProgSub11Minus:
 dec	TempH
 cpi	TempH, 254
 brne	_MPS11
 ldi	TempH, 143
 rjmp	_MPS11

_ProgSub11Plus:
 inc	TempH
 cpi	TempH, 144
 brne	_MPS11
 ldi	TempH, 255

_MPS11:
 st 	X, TempH
 cpi	TempH, 255
 breq	_TimerInactive
 
 mov	Temp, TempH
 call	DivTimer2HourTenMin

 mov	Temp, Di2
 cpi	Temp, '0'
 brne	_MPS111
 ldi	Temp, '?'							;clear leading zero

_MPS111:
 ldi	TempH, $10
 rcall	PutCharacter
 mov	Temp, Di1
 rcall	PutCharacter
 mov	Temp, Di0
 rcall	PutCharacter
 ldi	Temp, '0'
 rcall	PutCharacter
 ldi	TempH, $20
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 
 rcall	Show_TimerSetBar
 ret

_TimerInactive:
 ldi	Temp, ':'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, ':'
 rcall	PutCharacter
 ldi	Temp, ':'
 rcall	PutCharacter
 ldi	Temp, ':'
 rcall	PutCharacter
 ldi	TempH, $20
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ret

 
_ProgSub11Enter:
 rcall	ClearColon
 cbi	Status0, _MenuWork 
 
 lds	Temp, MenuLow
 andi	Temp, $01							;check for even submenu, 2nd part of a day timer
 brne	_ProgSub11Enter1
 ld		Temp, -X
 cpi	Temp, 255
 brne	_ProgSub11Enter1					;if both part of a day timer are inactive...
 adiw	XH:XL, 1
 ld		Temp, X+
 cpi	Temp, 255
 brne	_ProgSub11Enter1

 lds	Temp, MenuLow						;...set all day timers after one inactive Timer also to inactive state... 
 andi	Temp, $0F
 subi	Temp, 8
 ldi	TempH, 255

_ClearInactiveTimers:
 st		X+, TempH
 inc	Temp
 brne	_ClearInactiveTimers
 
 lds	Temp, MenuLow
 andi	Temp, $F0
 subi	Temp, -8
 sts	MenuLow, Temp						;...and jump forward to night timer submenu
 rjmp	_IncSubSub
 
_ProgSub11Enter1:
 lds	Temp, MenuLow
 andi	Temp, $0F
 cpi	Temp, 9								;check for last subsub menu (night timer)
 breq	_PS11Ent1	
 rjmp	_IncSubSub
 
_PS11Ent1:
 lds	Temp, MenuLow
 andi	Temp, $F0
 cpi	Temp, $80							;if block menu, then copy block data to selected days
 brcs	_PS11Ent2
 rcall	CopyTimerBlock						

_PS11Ent2:
 rcall	ClearSymbols
 lds	Temp, MenuLow
 andi	Temp, $F0
 sts	MenuLow, Temp
 rjmp	_IncSub

 
;............................................ 
Menu_ProgSub12:
 sbi	Status0, _MenuWork 
 push	Temp
 
 rcall	ClearSymbols
 ldi	Temp, LCD_OffHouse_SET
 ldi	TempH, $30
 rcall	PutSymbol
 rjmp	_MenuProg_Com
 
;............................................ 
Menu_ProgSub13:
 sbi	Status0, _MenuWork 
 push	Temp

 rcall	ClearSymbols
 ldi	Temp, LCD_InHouse_SET
 ldi	TempH, $30
 rcall	PutSymbol
 rjmp	_MenuProg_Com


;............................................ 
Menu_ProgSub14:
 sbi	Status0, _MenuWork 
 push	Temp
 
 rcall	ClearSymbols
 ldi	Temp, LCD_Moon_SET
 ldi	TempH, $30
 rcall	PutSymbol
 rjmp	_MenuProg_Com


 

;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 
Menu_Temp:
 ldi	Temp, 'T'
 ldi	TempH, $30
 rcall	PutCharacter
 ldi	Temp, 'E'
 rcall	PutCharacter
 ldi	Temp, 'M'
 rcall	PutCharacter
 ldi	Temp, 'P'
 rcall	PutCharacter
 ret

;...............................................................
Menu_TempSub1:
 sbi	Status0, _MenuWork  
  
 lds	XL, InHouseTempL
 lds	XH, InHouseTempH

 cpi	Temp, 1
 breq	_InHouseMinus
 cpi	Temp, 2
 breq	_InHousePlus
 cpi	Temp, 3
 breq	_InHouseEnter
 rjmp	_MTS11

_InHouseMinus:
 sbiw	XH:XL, 5
 cpi	XL, 40
 cpc	XH, Null
 brcc	_MTS11
 ldi	XL, 40
 clr	XH
 rjmp	_MTS11

_InHousePlus:
 adiw	XH:XL, 5
 ldi	Temp3, high (350)
 cpi	XL, low (350)
 cpc	XH, Temp3
 brcs	_MTS11
 ldi	XL, low	(350)
 ldi	XH, high (350)

_MTS11:
 sts	InHouseTempL, XL
 sts	InHouseTempH, XH
 movw	TempH:Temp, XH:XL
 call	DivHL
 mov	Temp, Di2
 ldi	TempH, $10
 rcall	PutCharacter

 rcall	SetPoint
 
 mov	Temp, Di1
 rcall	PutCharacter
 mov	Temp, Di0
 rcall	PutCharacter
 ldi	Temp, '@'
 rcall	PutCharacter
 ldi	TempH, $20
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter

 ldi	Temp, LCD_InHouse_SET
 ldi	TempH, $30
 rcall	PutSymbol
 ret

_InHouseEnter:
 cbi	Status0, _MenuWork  
 sts	MenuLow, Null
  
 rcall	ClearPoint
 rjmp	_GM_0


;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
Menu_Zeit:
 ldi	Temp, 'Z'
 ldi	TempH, $30
 rcall	PutCharacter
 ldi	Temp, 'E'
 rcall	PutCharacter
 ldi	Temp, 'I'
 rcall	PutCharacter
 ldi	Temp, 'T'
 rcall	PutCharacter
 ret


;...................................
;set Year value
Menu_ZeitSub1:
 sbi	Status0, _MenuWork  
 lds	TempH, Years

 cpi	Temp, 1
 breq	_YearsMinus
 cpi	Temp, 2
 breq	_YearsPlus
 cpi	Temp, 3
 breq	_YearsEnter
 rjmp	_MZS11

_YearsMinus:
 dec	TempH
 cpi	TempH, 255
 brne	_MZS11
 ldi	TempH, 99
 rjmp	_MZS11

_YearsPlus:
 inc	TempH
 cpi	TempH, 100
 brne	_MZS11 
 clr	TempH

_MZS11:
 sts	Years, TempH
 mov	Temp, TempH
 call	DivDez
  
 ldi	Temp, '2'
 ldi	TempH, $10
 rcall	PutCharacter
 ldi	Temp, '0'
 rcall	PutCharacter
 mov	Temp, Di1
 rcall	PutCharacter
 mov	Temp, Di0
 rcall	PutCharacter
 ldi	TempH, $20
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 rcall	CalcDayOfWeek
 ori	Temp, $80
 rcall	ClearWeekDays
 ldi	TempH, $30
 rcall	PutWeekDay
 ret

_YearsEnter:
 cbi	Status0, _MenuWork  
 rjmp	_IncSub


;....................................
;set Months value
 Menu_ZeitSub2:
 sbi	Status0, _MenuWork  
 lds	TempH, Months

 cpi	Temp, 1
 breq	_MonthsMinus
 cpi	Temp, 2
 breq	_MonthsPlus
 cpi	Temp, 3
 breq	_MonthsEnter
 rjmp	_MZS21

_MonthsMinus:
 dec	TempH
 cpi	TempH, 255
 brne	_MZS21
 ldi	TempH, 11
 rjmp	_MZS21

_MonthsPlus:
 inc	TempH
 cpi	TempH, 12
 brne	_MZS21 
 clr	TempH 

_MZS21:
 sts	Months, TempH
 
 lds	Temp, Days
 inc	Temp
 call	DivDez
  
 mov	Temp, Di1
 ldi	TempH, $30
 rcall	PutCharacter
 mov	Temp, Di0
 rcall	PutCharacter
 
 rcall	SetPoint

 ldi	Temp, '?'
 ldi	TempH, $22
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 
 lds	Temp, Months
 inc	Temp						;add 1 to month value to display 1...12
 call	DivDez
 
 mov	Temp, Di1
 ldi	TempH, $12
 rcall	PutCharacter
 mov	Temp, Di0
 rcall	PutCharacter
 rcall	CalcDayOfWeek
 ori	Temp, $80
 rcall	ClearWeekDays
 ldi	TempH, $30
 rcall	PutWeekDay
 ret

_MonthsEnter:
 cbi	Status0, _MenuWork  
 rjmp	_IncSub


;....................................
;set Days value
 Menu_ZeitSub3:
 sbi	Status0, _MenuWork  
 lds	Temp3, Days

 cpi	Temp, 1
 breq	_DaysMinus
 cpi	Temp, 2
 breq	_DaysPlus
 cpi	Temp, 3
 breq	_DaysEnter
 rjmp	_MZS31

_DaysMinus:
 rcall	MonthLastDay
 dec	Temp3
 cpi	Temp3, 255
 brne	_MZS31
 mov	Temp3, TempH
 dec	Temp3
 rjmp	_MZS31

_DaysPlus:
 rcall	MonthLastDay
 inc	Temp3
 cp		Temp3, TempH
 brne	_MZS31
 clr	Temp3

_MZS31:
 sts	Days, Temp3
 
 lds	Temp, Days
 inc	Temp							;add 1 to days value to display 1..31
 call	DivDez
  
 mov	Temp, Di1
 ldi	TempH, $10
 rcall	PutCharacter
 mov	Temp, Di0
 rcall	PutCharacter
 ldi	Temp, '?'
 ldi	TempH, $20
 rcall	PutCharacter
 ldi	Temp, '?'
 rcall	PutCharacter
 
 lds	Temp, Months
 inc	Temp							;add 1 to month value to display 1...12
 call	DivDez
 
 mov	Temp, Di1
 ldi	TempH, $32
 call	PutCharacter
 mov	Temp, Di0
 call	PutCharacter
 rcall	CalcDayOfWeek
 ori	Temp, $80
 rcall	ClearWeekDays
 ldi	TempH, $30
 rcall	PutWeekDay
 ret

_DaysEnter: 
 cbi	Status0, _MenuWork  
 rcall	ClearPoint
 rjmp	_IncSub



;....................................
;set Hours value
 Menu_ZeitSub4:
 sbi	Status0, _MenuWork  
 lds	TempH, Hours

 cpi	Temp, 1
 breq	_HoursMinus
 cpi	Temp, 2
 breq	_HoursPlus
 cpi	Temp, 3
 breq	_HoursEnter
 rjmp	_MZS41

_HoursMinus:
 dec	TempH
 cpi	TempH, 255
 brne	_MZS41
 ldi	TempH, 23
 rjmp	_MZS41

_HoursPlus:
 inc	TempH
 cpi	TempH, 24
 brne	_MZS41 
 clr	TempH 

_MZS41:
 sts	Hours, TempH
 
 mov	Temp, TempH
 call	DivDez
  
 mov	Temp, Di1
 cpi	Temp, '0'
 brne	_MSZ411
 ldi	Temp, '?'						;clear leading zero

_MSZ411:
 ldi	TempH, $10
 call	PutCharacter
 mov	Temp, Di0
 call	PutCharacter
 
 ldi	Temp, '?'
 ldi	TempH, $20
 call	PutCharacter
 ldi	Temp, '?'
 call	PutCharacter
 
 rcall	SetColon

 lds	Temp, Minutes
 call	DivDez
 
 mov	Temp, Di1
 ldi	TempH, $32
 call	PutCharacter
 mov	Temp, Di0
 call	PutCharacter
 ret

_HoursEnter:
 cbi	Status0, _MenuWork  
 rjmp	_IncSub
 

 ;....................................
;set Minutes value
 Menu_ZeitSub5:
 sbi	Status0, _MenuWork  
 lds	TempH, Minutes

 cpi	Temp, 1
 breq	_MinutesMinus
 cpi	Temp, 2
 breq	_MinutesPlus
 cpi	Temp, 3
 breq	_MinutesEnter
 rjmp	_MZS51

_MinutesMinus:
 dec	TempH
 cpi	TempH, 255
 brne	_MZS51
 ldi	TempH, 59
 rjmp	_MZS51

_MinutesPlus:
 inc	TempH
 cpi	TempH, 60
 brne	_MZS51 
 clr	TempH 

_MZS51:
 sts	Minutes, TempH
 
 lds	Temp, Hours
 call	DivDez
  
 mov	Temp, Di1
 cpi	Temp, '0'
 brne	_MSZ511
 ldi	Temp, '?'						;clear leading zero

_MSZ511:
 ldi	TempH, $30
 call	PutCharacter
 mov	Temp, Di0
 call	PutCharacter

 lds	Temp, Minutes
 call	DivDez
 
 mov	Temp, Di1
 ldi	TempH, $12
 call	PutCharacter
 mov	Temp, Di0
 call	PutCharacter
 ldi	Temp, '?'
 ldi	TempH, $22
 call	PutCharacter
 ldi	Temp, '?'
 call	PutCharacter
 ret

_MinutesEnter:
 cbi	Status0, _MenuWork  
 sts	MenuLow, Null
 rcall	ClearColon
 rjmp	_GM_0
 
 



;---------------------------------------------------
Menu_Fens:
 ldi	Temp, 'F'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'E'
 call	PutCharacter
 ldi	Temp, 'N'
 call	PutCharacter
 ldi	Temp, 'S'
 call	PutCharacter
 ret

Menu_FensSub1:
 ldi	Temp, 'H'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'O'
 call	PutCharacter
 ldi	Temp, 'C'
 call	PutCharacter
 ldi	Temp, 'H'
 call	PutCharacter
 ret


Menu_FensSub2:
 ldi	Temp, 'M'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'I'
 call	PutCharacter
 ldi	Temp, 'T'
 call	PutCharacter
 ldi	Temp, 'T'
 call	PutCharacter
 ret

Menu_FensSub3:
 ldi	Temp, 'N'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'I'
 call	PutCharacter
 ldi	Temp, 'E'
 call	PutCharacter
 ldi	Temp, 'D'
 call	PutCharacter
 ret

Menu_FensSub11:
 lds	Temp, MenuLow
 andi	Temp, $0F
 call	DivDez
 mov	Temp, Di0
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, '0'
 call	PutCharacter
 ldi	Temp, '?'
 call	PutCharacter
 ldi	Temp, 'M'
 call	PutCharacter
 ret

Menu_Reset:
 ldi	Temp, 'R'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'E'
 call	PutCharacter
 ldi	Temp, 'S'
 call	PutCharacter
 ldi	Temp, '?'
 call	PutCharacter
 ret

Menu_ResetSub1:
 ldi	Temp, '?'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'O'
 call	PutCharacter
 ldi	Temp, 'K'
 call	PutCharacter
 ldi	Temp, '?'
 call	PutCharacter
 ret

Menu_Adapt:
 ldi	Temp, 'A'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'D'
 call	PutCharacter
 ldi	Temp, 'A'
 call	PutCharacter
 ldi	Temp, 'P'
 call	PutCharacter
 ret

Menu_AdaptSub1:
 call	StartMain
 cbi	Status0, _MenuOn
 ret

Menu_Urla:
 ldi	Temp, 'U'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'R'
 call	PutCharacter
 ldi	Temp, 'L'
 call	PutCharacter
 ldi	Temp, 'A'
 call	PutCharacter
 ret

Menu_UrlaSub1:
 ldi	Temp, 'D'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'A'
 call	PutCharacter
 ldi	Temp, 'T'
 call	PutCharacter
 ldi	Temp, 'E'
 call	PutCharacter
 ret
 
Menu_Inst:
 ldi	Temp, 'I'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'N'
 call	PutCharacter
 ldi	Temp, 'S'
 call	PutCharacter
 ldi	Temp, 'T'
 call	PutCharacter
 ret

Menu_InstSub1:
 ldi	Temp, '<'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, '<'
 call	PutCharacter
 ldi	Temp, '<'
 call	PutCharacter
 ldi	Temp, '<'
 call	PutCharacter
 ret

Menu_Offs:
 ldi	Temp, 'O'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'F'
 call	PutCharacter
 ldi	Temp, 'F'
 call	PutCharacter
 ldi	Temp, 'S'
 call	PutCharacter
 ret

Menu_OffsSub1:
 ldi	Temp, '+'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, '0'
 call	PutCharacter
 ldi	Temp, '@'
 call	PutCharacter
 ldi	Temp, '?'
 call	PutCharacter
 ret

Menu_Debug:
 ldi	Temp, 'D'
 ldi	TempH, $30
 call	PutCharacter
 ldi	Temp, 'B'
 call	PutCharacter
 ldi	Temp, 'U'
 call	PutCharacter
 ldi	Temp, 'G'
 call	PutCharacter
 ret


Menu_Dbg_FW:
 ldi	Temp, 'F'
 ldi	TempH, $10
 call	PutCharacter
 ldi	Temp, 'I'
 call	PutCharacter
 ldi	Temp, 'R'
 call	PutCharacter
 ldi	Temp, 'M'
 call	PutCharacter

 ldi	Temp, FW_Version
 call	DivDez
 
 ldi	Temp, 'V'
 ldi	TempH, $20
 call	PutCharacter
 mov	Temp, Di2
 call	PutCharacter
 mov	Temp, Di1
 call	PutCharacter
 mov	Temp, Di0
 call	PutCharacter
 ret

Menu_Dbg_1:
 ldi	Temp, 'F'
 ldi	TempH, $10
 call	PutCharacter
 ldi	Temp, 'U'
 call	PutCharacter
 ldi	Temp, 'Z'
 call	PutCharacter
 ldi	Temp, 'Z'
 call	PutCharacter
 lds	Temp, FuzzyVal
 call	DivDez
 ldi	Temp, '?'
 ldi	TempH, $20
 call	PutCharacter
 ldi	Temp, '?'
 call	PutCharacter
 mov	Temp, Di1
 call	PutCharacter
 mov	Temp, Di0
 call	PutCharacter
 ret

Menu_Dbg_2:
 ldi	Temp, 'P'
 ldi	TempH, $10
 call	PutCharacter
 ldi	Temp, 'O'
 call	PutCharacter
 ldi	Temp, 'S'
 call	PutCharacter
 ldi	Temp, 'I'
 call	PutCharacter
 lds	Temp, PositionL
 lds	TempH, PositionH
 call	DivHL
 mov	Temp, Di3
 ldi	TempH, $20
 call	PutCharacter
 mov	Temp, Di2
 call	PutCharacter
 mov	Temp, Di1
 call	PutCharacter
 mov	Temp, Di0
 call	PutCharacter
 ret

Menu_Dbg_3:
 ldi	Temp, 'V'
 ldi	TempH, $10
 call	PutCharacter
 ldi	Temp, 'T'
 call	PutCharacter
 ldi	Temp, 'O'
 call	PutCharacter
 ldi	Temp, 'P'
 call	PutCharacter
 lds	Temp, ValveTopL
 lds	TempH, ValveTopH
 call	DivHL
 mov	Temp, Di3
 ldi	TempH, $20
 call	PutCharacter
 mov	Temp, Di2
 call	PutCharacter
 mov	Temp, Di1
 call	PutCharacter
 mov	Temp, Di0
 call	PutCharacter
 ret

Menu_Dbg_4:
 ldi	Temp, 'R'
 ldi	TempH, $10
 call	PutCharacter
 ldi	Temp, 'W'
 call	PutCharacter
 ldi	Temp, 'A'
 call	PutCharacter
 ldi	Temp, 'Y'
 call	PutCharacter
 lds	Temp, RegWay
 call	DivDez
 ldi	Temp, '?'
 ldi	TempH, $20
 call	PutCharacter
 mov	Temp, Di2
 call	PutCharacter
 mov	Temp, Di1
 call	PutCharacter
 mov	Temp, Di0
 call	PutCharacter
 ret



;------------------------------------------
CopyTimerBlock:
 cpi	Temp, $80
 breq	_Copy1_5
 cpi	Temp, $90
 breq	_Copy1_6
 cpi	Temp, $A0
 breq	_Copy1_7
 ret

_Copy1_5:
 ldi	Temp3, 5
 ldi	XL, low (DailyTimer+63)
 ldi	XH, high (DailyTimer+63)
 rjmp	_CTB1

_Copy1_6:
 ldi	Temp3, 6
 ldi	XL, low (DailyTimer+72)
 ldi	XH, high (DailyTimer+72)
 rjmp	_CTB1

_Copy1_7:
 ldi	Temp3, 7
 ldi	XL, low (DailyTimer+81)
 ldi	XH, high (DailyTimer+81)

_CTB1:
 ldi	YL, low(DailyTimer)
 ldi	YH, high(DailyTimer)

_CTB2:
 ldi	TempH, 9

_CTBLoop:
 ld		Temp, X+ 
 st		Y+, Temp
 dec	TempH
 brne	_CTBLoop
 
 sbiw	XH:XL, 9
 dec	Temp3
 brne	_CTB2
 ret