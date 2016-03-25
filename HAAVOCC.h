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

#define MED_SPEED_INIT              1200 
#define SLOW_SPEED_INIT             675 
#define SUPER_SLOW_SPEED_INIT       505 

/// TEST FOR MIN AND MAX PWM VALUES
#define MAX_SPEED_PWM       1500
#define MIN_SPEED_PWM       300


#define MED_SPEED           150
#define SLOW_SPEED          90
#define SUPER_SLOW_SPEED    70

#define RIGHT_TURN          650
#define FULL_TURN           1300
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
void MotorSpeedCtrl( uint32 LSpeed, uint32 RSpeed );
void MotorDirectionCtrl( uint8 LDirection, uint8 RDirection);
uint16 PI( uint16 ActlEncoder, uint8 Motor );
void SetSpeed( uint32 Speed);
void SetDirection( uint32 Direction);
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
    SUPER_SLOW,
    SLOW,
    MED,
};

enum DIRECTION
{
    FORWARD,
    REVERSE,
    LEFT_90,
    RIGHT_90,
    HALF_TURN
};

enum WHEEL_DIRECTION
{
    RVS = 0,
    FWD = 1
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
uint32 TargetEncoder;

float M1Integral;
float M2Integral;
// DEBUG
uint32 TEST4;
uint32 CatchUp;
uint32 Fixed;

uint32 M1Distance;
uint32 M2Distance;
uint8 TurnFlag;
uint32 TurnCnt;

uint8 M2Faster;
uint8 M1Faster;

// Sensor
uint16 i;
float val;
uint32 SensorCnt;
float Inches[40];
uint16 ADC10[40];

float inch;
uint32 distance;
sint32 distanceDiff;
uint8 SpeedUp;

#endif	/* HAVVOCC_H */
