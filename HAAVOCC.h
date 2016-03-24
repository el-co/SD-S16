//****************************************************************************
//Filename:		HAAVOCC.h
//
//Description:	HAAVOCC Header file
//
//Date:         03.12.16
//
//Complier:		XC32
//
//****************************************************************************

#ifndef HAVVOCC_H
#define	HAVVOCC_H

#include <xc.h>
#include <p32xxxx.h>   // include chip specific header file
#include <plib.h>      // include peripheral library functions
#include <stdlib.h>
#include <sys/attribs.h>

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef signed short sint16;
typedef unsigned int uint32;
typedef signed int sint32;

#define FAST_SPEED_INIT     0x6D6 //70% 1750 //65% 1625
#define MED_SPEED_INIT      0x4E2 //50% 1250
#define SLOW_SPEED_INIT     675 //20% 500

/// TEST FOR MIN AND MAX PWM VALUES
#define MAX_SPEED_PWM       0x6D6
#define MIN_SPEED_PWM       0xC8

#define FAST_SPEED  276
#define MED_SPEED   200
#define SLOW_SPEED  90
#define STEP_SPEED  50
//****************************************************************************
//
//
//                              Global Functions
//
//
//****************************************************************************
//****************************************************************************
//
//                          Initializing Functions     
//
//****************************************************************************
void Init(void);
void GPIOInit(void);
void TimerInit(void);
void PWMInit(void);
void ADCInit(void);
void ICInit(void);

//****************************************************************************
//
//                          Motor Control Functions     
//
//****************************************************************************
void MotorControl( uint32 LSpeed, uint32 RSpeed, uint8 LDirection, uint8 RDirection );
uint16 P( uint16 ActlEncoder, uint8 Motor );
void SetSpeed( uint32 Speed);
void SensorCalc();
void DebugFlag();

//****************************************************************************
// 
//                              enum Variables
//
//****************************************************************************
enum HAAVOCC_STATES
{
    START,
    NAVIGATE,
    FIRE_DETECTED,
    FIRE_VERIFY,
    FIRE_EXTINGUISH,
    IDLE
};

enum SPEEDS
{
    OFF,
    SLOW,
    MED,
    FAST
};

enum DIRECTION
{
    RVRS = 0,
    FWRD = 1
};

//****************************************************************************
//
//                              Global Variables
//
//****************************************************************************
uint8 Map[84][30];
uint8 State;

uint16 AN1ADC;
uint16 AN4ADC;
uint16 AN5ADC;
uint16 AN9ADC;
uint16 AN10ADC;
uint16 ANADC;
uint8 SensorEvalFlag;

uint32 M1PosEdgeCnt;
uint32 M2PosEdgeCnt;

uint16 IC4PosEdgeTime;
uint16 IC4NegEdgeTime;
uint16 DC;
float Hz;
uint32 TM364PS;
float CalcSensPer;
float Input;
uint8 USSensorFlag;
uint32 IC4EdgeCnt;
uint32 PersistantBuffer;

uint8 AdjustSpeedFlag;
uint16 Motor2Speed;
uint16 Motor1Speed;
uint32 StepEncoder;
uint32 TargetEncoder;
uint32 StepCnt;

float M1Integral;
float M2Integral;
// DEBUG
uint32 ADCCNT;
uint32 TEST2;
uint32 TEST4;
uint32 TestTime;
uint32 TTCnt;
uint32 TTFlag;
uint32 M1PWM;
uint32 M2PWM;
uint32 TE;
uint32 CatchUp;
uint32 Fixed;
sint32 LastErrorM1;
sint32 LastErrorM2;
sint32 LastAdjustM1;
sint32 LastAdjustM2;
sint32 LastEncoderM1;
sint32 LastEncoderM2;

uint8 M1Adjust;
uint8 M2Adjust;


uint8 ADCCH;
 
uint32 M1Distance;
uint32 M2Distance;
uint8 TurnFlag;

uint8 M2Faster;
uint8 M1Faster;

uint8 T2int;
uint8 T3int;
uint8 T4int;
uint8 AD1int;
uint8 IC2int;
uint8 IC3int;
uint8 IC4int;

// Sensor
uint16 i;
uint32 cnt;
float val;
float Inches[40];
uint16 ADC10[40];

float inch;
uint32 distance;
sint32 distanceDiff;

#endif	/* HAVVOCC_H */
