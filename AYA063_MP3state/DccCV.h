//--------------------------------------------------------------------------------
//
// SMFDEasy_AYA071 LED ファンクションデコーダスケッチ NMRA認証
// Copyright(C)'2021 Ayanosuke(Maison de DCC)
// [DccCV.h]
// AYA071-2
//
// PIN_F0_F 0      // Atiny85 PB0(5pin)O7 analogwrite head light
// PIN_F0_R 1      // Atiny85 PB1(6pin)O6 analogwrite tail light
// PIN_AUX2 3      // Atint85 PB3(2pin)O3             sign light
// PIN_AUX1 4      // Atiny85 PB4(3pin)O2 analogwrite room light
//
// http://maison-dcc.sblo.jp/ http://dcc.client.jp/ http://ayabu.blog.shinobi.jp/
// https://twitter.com/masashi_214
//
// DCC電子工作連合のメンバーです
// https://desktopstation.net/tmi/ https://desktopstation.net/bb/index.php
//
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
//
//--------------------------------------------------------------------------------

#ifndef _DccCV_h_
#define _DccCV_h_

#define DECODER_ADDRESS 3

#define ON 1
#define OFF 0
#define UP 1
#define DOWN 0

#define DECODER_ADDRESS 3
#define DCC_ACK_PIN 0   // Atiny85 PB0(5pin) if defined enables the ACK pin functionality. Comment out to disable.
//                      // Atiny85 DCCin(7pin)
#define PIN_AUX1 1      // Atiny85 PB1(6pin)O6 analogwrite AUX1 light F1
#define PIN_AUX2 4      // Atiny85 PB4(3pin)O2 analogwrite AUX2
#define PIN_F0_F 0      // Atiny85 PB0(5pin)O7 analogwrite head light F0
#define PIN_F0_R 3      // Atint85 PB3(2pin)O3             tail light F0
//#define PIN_F0_F 0      // Atiny85 PB0(5pin)O7 analogwrite head light
//#define PIN_F0_R 1      // Atiny85 PB1(6pin)O6 analogwrite tail light
//#define PIN_AUX2 3      // Atint85 PB3(2pin)O3             sign light
//#define PIN_AUX1 4      // Atiny85 PB4(3pin)O2 analogwrite room light

#define MAX_PWMDUTY 255
#define MID_PWMDUTY 10

#define CV_VSTART    2
#define CV_ACCRATIO   3
#define CV_DECCRATIO  4
#define CV_F0_FORWARD 33
#define CV_F0_BACK 34
#define CV_F1 35
#define CV_F2 36
#define CV_F3 37
#define CV_F4 38
#define CV_F5 39
#define CV_F6 40
#define CV_F7 41
#define CV_F8 42
#define CV_F9 43
#define CV_F10 44
#define CV_F11 45
#define CV_F12 46
#define CV_49_F0_FORWARD_LIGHT 49
#define CV_DIMMING_SPEED 50
#define CV_DIMMING_LIGHT_QUANTITY 51
#define CV_HEADLIGHT_DIMMING 52
#define CV_ROOM_DIMMING 53

#define CV_dummy 69         // dummy
#define MAN_ID_NUMBER 166   // Manufacture ID //
#define MAN_VER_NUMBER  01  // Release Ver CV07 //

//Task Schedule
unsigned long gPreviousL5 = 0;

//進行方向
uint8_t gDirection = 128;

//モータ制御関連の変数
uint32_t gSpeedRef = 1;

//Function State
uint8_t gState_F0 = 0;
uint8_t gState_F1 = 0;
uint8_t gState_F2 = 0;
uint8_t gState_F3 = 0;
uint8_t gState_F4 = 0;
uint8_t gState_F5 = 0;
uint8_t gState_F6 = 0;
uint8_t gState_F7 = 0;
uint8_t gState_F8 = 0;
uint8_t gState_F9 = 0;
uint8_t gState_F10 = 0;
uint8_t gState_F11 = 0; 
uint8_t gState_F12 = 0;
uint8_t gState_F13 = 0;
uint8_t gState_F14 = 0; 
uint8_t gState_F15 = 0;
uint8_t gState_F16 = 0;
uint8_t gState_Function = 0;

//ファンクションの変数
uint8_t fn_bit_f0 = 0;
uint8_t fn_bit_f1 = 0;
uint8_t fn_bit_f2 = 0;
uint8_t fn_bit_f3 = 0;
uint8_t fn_bit_f4 = 0;
uint8_t fn_bit_f5 = 0;
uint8_t fn_bit_f6 = 0;
uint8_t fn_bit_f7 = 0;
uint8_t fn_bit_f8 = 0;
uint8_t fn_bit_f9 = 0;
uint8_t fn_bit_f10 = 0;
uint8_t fn_bit_f11 = 0;
uint8_t fn_bit_f12 = 0;
uint8_t fn_bit_f13 = 0;
uint8_t fn_bit_f14 = 0;
uint8_t fn_bit_f15 = 0;
uint8_t fn_bit_f16 = 0;
uint8_t fn_bit_f17 = 0;
uint8_t fn_bit_f18 = 0;
uint8_t fn_bit_f19 = 0;
uint8_t fn_bit_f20 = 0;
uint8_t fn_bit_f21 = 0;
uint8_t fn_bit_f22 = 0;
uint8_t fn_bit_f23 = 0;
uint8_t fn_bit_f24 = 0;
uint8_t fn_bit_f25 = 0;
uint8_t fn_bit_f26 = 0;
uint8_t fn_bit_f27 = 0;
uint8_t fn_bit_f28 = 0;

//CV related
uint8_t gCV1_SAddr = 3; 
uint8_t gCVx_LAddr = 3;
uint8_t gCV29_Vstart = 0;
uint8_t gCV29Direction = 0;
uint8_t gCV49_fx = 20;
uint8_t gCV50_DimmingSpeed = 1;
uint8_t gCV51_DimmingLightQuantity = 10;
uint8_t gCV52_HeadLighDimming = 250;
uint8_t gCV53_RoomDimming = 128;
  
#endif
