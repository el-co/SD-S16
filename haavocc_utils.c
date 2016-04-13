//****************************************************************************
//Filename:		haavocc_utils.c
//
//Description:	HAAVOCC Utility functions
//
//Date:         03.12.16
//
//Complier:		XC32
//
//****************************************************************************

#include "HAAVOCC.h"

//****************************************************************************
// Function:    Init
// 
// Description: Initializes variables and other Init functions.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void Init(void)
{
    INTEnableSystemMultiVectoredInt(); 
    
    State = IDLE; 
 
    GPIOInit();   
    TimerInit();
    ADCInit();
    ICInit();
    PWMInit();
    SetSpeed( OFF );    
    SetDirection( FORWARD );      
    MotorSpeedCtrl( Motor1Speed, Motor2Speed );   
    
    M1Distance = 0;
    M2Distance = 0;
    
    Extinguish = 0;
}

//****************************************************************************
// Function:    GPIOInit
// 
// Description: Initializes IO Pins as Analog/Digital and Input/Output.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void GPIOInit(void)
{
    // LED Pins
    TRISAbits.TRISA7 = 0;       // RA7 Output   
    TRISAbits.TRISA9 = 0;       // RA9 Output   
    TRISAbits.TRISA10 = 0;      // RA10 Output   
    TRISCbits.TRISC7 = 0;       // RC7 Output   
    TRISCbits.TRISC8 = 0;       // RC8 Output   

    // ADC Pins
    // Flame Sensors
    ANSELAbits.ANSA0 = 1;       // RA0 Analog  - AN0
    ANSELAbits.ANSA1 = 1;       // RA1 Analog  - AN1
    ANSELBbits.ANSB2 = 1;       // RB2 Analog  - AN4
    ANSELBbits.ANSB3 = 1;       // RB3 Analog  - AN5
    ANSELCbits.ANSC0 = 1;       // RC0 Analog  - AN6
    TRISAbits.TRISA0 = 1;       // RA0 Input  - AN0        
    TRISAbits.TRISA1 = 1;       // RA1 Input  - AN1     
    TRISBbits.TRISB2 = 1;       // RB2 Input  - AN4
    TRISBbits.TRISB3 = 1;       // RB3 Input  - AN5
    TRISCbits.TRISC0 = 1;       // RC0 Input  - AN6 
    
    // IR Sensors
    ANSELBbits.ANSB13 = 1;      // RB13 Analog - AN11
    ANSELBbits.ANSB14 = 1;      // RB14 Analog - AN10
    ANSELBbits.ANSB15 = 1;      // RB15 Analog - AN9
    ANSELCbits.ANSC3 = 1;       // RC3 Analog  - AN12
    TRISBbits.TRISB13 = 1;      // RB13 Input - AN11
    TRISBbits.TRISB14 = 1;      // RB14 Input - AN10
    TRISBbits.TRISB15 = 1;      // RB15 Input - AN9            
    TRISCbits.TRISC3 = 1;       // RC3 Input  - AN12   
    
    // PWM Pins
    TRISAbits.TRISA2 = 0;       //  RPA2 OC4    - PWM2
    TRISAbits.TRISA3 = 0;    	//  RPA3 OC3    - PWM1
    TRISAbits.TRISA4 = 0;       //  RA4         - INB1
    TRISBbits.TRISB4 = 0;		//  RB4         - INA1
    TRISBbits.TRISB7 = 0;       //  RB7         - INA2
    TRISBbits.TRISB10 = 0;		//  RB10        - INB2
    
    // IC Pins
    TRISBbits.TRISB5 = 1;       // RB5 Input
    TRISCbits.TRISC5 = 1;       // RC5 Input
    TRISCbits.TRISC6 = 1;       // RC6 Input   
    
    // Solenoid
    TRISCbits.TRISC4 = 0;    	//  RPC4 Output

}

//****************************************************************************
// Function:    TimerInit
// 
// Description: Initializes Timers 2, 3 and 4.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void TimerInit(void)
{
    // TIMER 2
    PR2 = 0x09C4;       // 16KHz 62.5us
                        // TMR period for 1ms  1kHz 0x9CE0
    TMR2 = 0;           // Reset TMR2 to 0        
    
    // Timer 2 Interrupt Config
    IEC0bits.T2IE = 0;  // TMR2 Interrupt Disable 
    IFS0bits.T2IF = 0;  // TMR2 Interrupt Flag Off
    IPC2bits.T2IP = 2;  // TMR2 Interrupt Priority
//    IPC2bits.T2IS = 1;  // TMR2 Interrupt SubPriority
    IEC0bits.T2IE = 1;  // TMR2 Interrupt Enable         
 
    T2CON = 0x8000;     // TMR2 Start
    
    // TIMER 3
    T3CONbits.TCKPS = 0b110;    // 1:64 prescaler
    PR3 = 0x77A1;               // 49ms 0x77A1
                                // 98 0xEF42
    TMR3 = 0;                   // Reset TMR3 to 0        
    
    // Timer 3 Interrupt Config
    IEC0bits.T3IE = 0;  // TMR3 Interrupt Disable 
    IFS0bits.T3IF = 0;  // TMR3 Interrupt Flag Off
    IPC3bits.T3IP = 2;  // TMR3 Interrupt Priority
//    IPC3bits.T3IS = 1;  // TMR3 Interrupt SubPriority
    IEC0bits.T3IE = 1;  // TMR3 Interrupt Enable         
 
    T3CONbits.ON = 1;   // TMR3 Start
    
    // TIMER 4
    T4CONbits.TCKPS = 0b111;    // 1:256 prescaler
    PR4 = 0x2BF2;               // 10Hz 80ms 0x30D4 
    
    TMR4 = 0;
    T4CONbits.ON = 1;
    
    IEC0bits.T4IE = 0;  // TMR4 Interrupt Disable 
    IFS0bits.T4IF = 0;  // TMR4 Interrupt Flag Off
    IPC4bits.T4IP = 2;  // TMR4 Interrupt Priority
//    IPC4bits.T4IS = 1;  // TMR4 Interrupt SubPriority
    IEC0bits.T4IE = 1;  // TMR4 Interrupt Enable    
}

//****************************************************************************
// Function:    PWMInit
// 
// Description: Initializes OC3 and OC4 as PWM.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void PWMInit(void)
{
    RPA2Rbits.RPA2R = 0b0101;   //PPS OC4 for A2 (pin 30)
    RPA3Rbits.RPA3R = 0b0101;   //PPS OC3 for A3 (pin 31)
    
    OC3CON = 0x0000;            // OC3 Disable
    OC3CON = 0x0006;            // OCM PWM mode   
    OC3RS = 0;             // OC3 Secondary Compare Register
                                // 0x1F4 0x4E2 0x659 0X6D6 0x753 0x7D0
                                // 0x7FF //0x698 //0x392 //0x3FF 
    OC4CON = 0x0000;            // OC4 Disable
    OC4CON = 0x0006;            // OCM PWM mode   
    OC4RS = 0;             // OC4 Secondary Compare Register
  
    OC3CONSET = 0x8000;         // OC3 Enable
    OC4CONSET = 0x8000;         // OC4 Enable   
}

//****************************************************************************
// Function:    ADCInit
// 
// Description: Initializes AD1 as auto convert and scan, with 5 channels.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void ADCInit(void)
{
    AD1CON1 = 0x0000;           // AD1 Enable  
    //0x00E0 1110 0000
    AD1CON1bits.SSRC = 0b010;   // Auto Convert
    AD1CON1bits.CLRASAM = 1;    // Stop conversions when the first ADC interrupt is generated
                                // Will also clear ASAM bit  
    AD1CON2bits.CSCNA = 1;      // Scan Inputs
    AD1CON2bits.SMPI = 8;       // Interrupts at the completion of conversion for each 9th sample/convert sequence
    
//    AD1CON3bits.SAMC = 31;      // Auto-Sample Time bits - 31 TAD
//    AD1CON3bits.ADCS = 1;       // ADC Conversion Clock Select bits - TPB * 2 * (ADCS + 1) = TAD
    
    AD1CHS  = 0;                // AD1 INPUT SELECT REGISTER
                                // Not needed for auto-sampling
    AD1CSSLbits.CSSL0 = 1;      // Select ANx for input scan - AN0 (pin 19)
    AD1CSSLbits.CSSL1 = 1;      // Select ANx for input scan - AN1 (pin 20)
    AD1CSSLbits.CSSL4 = 1;      // Select ANx for input scan - AN4 (pin 23)
    AD1CSSLbits.CSSL5 = 1;      // Select ANx for input scan - AN5 (pin 24)
    AD1CSSLbits.CSSL6 = 1;      // Select ANx for input scan - AN6 (pin 25)
    AD1CSSLbits.CSSL9 = 1;      // Select ANx for input scan - AN9  (pin 15)
    AD1CSSLbits.CSSL10 = 1;      // Select ANx for input scan - AN10 (pin 14)
    AD1CSSLbits.CSSL11 = 1;      // Select ANx for input scan - AN11 (pin 11)
    AD1CSSLbits.CSSL12 = 1;      // Select ANx for input scan - AN12 (pin 36)
     
    // ADC 1 Interrupt Config
    IEC0bits.AD1IE = 0 ;        // AD1 Interrupt Enable Off
    IFS0bits.AD1IF = 0 ;        // AD1 Interrupt Flag Off
    IPC5bits.AD1IP = 3;         // AD1 Interrupt Priority
//    IPC5bits.AD1IS = 1;         // AD1 Interrupt SubPriority

    IEC0bits.AD1IE = 1;         // AD1 Interrupt Enable On   
    
    AD1CON1bits.ASAM = 1;       // ADC Sample Auto-Start   
//    AD1CON1bits.SAMP = 1;    
    AD1CON1bits.ADON = 1;       // AD1 Enable
    
}

//****************************************************************************
// Function:    ICInit
//
// Description: Initializes IC2 and IC3 as to capture posedge of encoders.
//              Initializes IC4 to capture pos and negedges.    
//
// Params:      void
//
// Return:      void
//
//****************************************************************************
void ICInit(void)
{
    // IC4 Motor 2
    IC4R = 7;                   // Input Pin Selection - C5 (pin 38)
    IC4CONbits.FEDGE = 1;       // Capture rising edge first
    IC4CONbits.ICTMR = 0;       // Timer 3 Select
    IC4CONbits.ICM = 0b011;     // Input Capture Mode - Simple Capture Event - Every rising edge
    IC4CONbits.ICI = 0b00;      // Interrupt on every capture event
    
    IEC0bits.IC4IE = 0;         // IC4 Interrupt Enable Off
    IFS0bits.IC4IF = 0;         // IC4 Interrupt Flag Off
    IPC4bits.IC4IP = 4;         // IC4 Interrupt Priority
//    IPC4bits.IC4IS = 2;         // IC4 Interrupt SubPriority
    
    // IC3 Motor 1
    IC3R = 1;                   // Input Pin Selection - B5 (pin 41)
    IC3CONbits.FEDGE = 1;       // Capture rising edge first
    IC3CONbits.ICTMR = 0;       // Timer 3 Select
    IC3CONbits.ICM = 0b011;     // Input Capture Mode - Simple Capture Event - Every rising edge
    IC3CONbits.ICI = 0b00;      // Interrupt on every capture event
    
    IEC0bits.IC3IE = 0;         // IC3 Interrupt Enable Off
    IFS0bits.IC3IF = 0;         // IC3 Interrupt Flag Off
    IPC3bits.IC3IP = 4;         // IC3 Interrupt Priority
//    IPC3bits.IC3IS = 2;         // IC3 Interrupt SubPriority

    // IC1
    IC1R = 5;                   // Input Pin Selection - C6 (pin 2)
    IC1CONbits.FEDGE = 1;       // Capture rising edge first
    IC1CONbits.ICTMR = 0;       // Timer 3 Select
    IC1CONbits.ICM = 0b110;     // Input Capture Mode - every edge, specified edge first
    IC1CONbits.ICI = 0b00;      // Interrupt on every capture event
    
    IEC0bits.IC1IE = 0;         // IC1 Interrupt Enable Off
    IFS0bits.IC1IF = 0;         // IC1 Interrupt Flag Off
    IPC1bits.IC1IP = 4;         // IC1 Interrupt Priority
//    IPC1bits.IC1IS = 4;         // IC1 Interrupt SubPriority    
    
    // ENABLE
    IEC0bits.IC1IE = 1;         // IC1 Interrupt Enable
    IEC0bits.IC3IE = 1;         // IC3 Interrupt Enable
    IEC0bits.IC4IE = 1;         // IC4 Interrupt Enable
    
    IC1CONbits.ON = 1;          // IC1 Enable    
    IC3CONbits.ON = 1;          // IC3 Enable
    IC4CONbits.ON = 1;          // IC4 Enable    
}

//****************************************************************************
// Function:    MotorSpeedCtrl
//
// Description: Initializes variables and other Init functions.
//
// Params:      LSpeed - PWM Duty cycle to control left motor
//              RSpeed - PWM Duty cycle to control right motor
//
// Return:      void
//
//****************************************************************************
void MotorSpeedCtrl( uint32 M1Speed, uint32 M2Speed )
{    
    // VERIFY
    OC4RS = M1Speed; // Slower motor            
    OC3RS = M2Speed;  
}

//****************************************************************************
// Function:    MotorDirectionCtrl
//
// Description: Initializes variables and other Init functions.
//
// Params:      LDirection - Sets direction of left motor
//              RDirection - Sets direction of right motor
//
// Return:      void
//
//****************************************************************************
void MotorDirectionCtrl( uint8 LDirection, uint8 RDirection)
{
    switch (LDirection)
    {
        case RVS:
            LATAbits.LATA4 = FWD;       // pin 34
            LATBbits.LATB4 = RVS;       // pin 33                
            break;
        case FWD:
            LATAbits.LATA4 = RVS;       // pin 34
            LATBbits.LATB4 = FWD;       // pin 33    
            break;
        case STLL:
            LATAbits.LATA4 = FWD;       // pin 34
            LATBbits.LATB4 = FWD;       // pin 33                  
            break;
        case IGN:               
            break;
        default:
            break;
    }        
 
    switch (RDirection)
    {
        case RVS:
            LATBbits.LATB7 = FWD;       // pin 43
            LATBbits.LATB10 = RVS;      // pin 8                
            break;
        case FWD:
            LATBbits.LATB7 = RVS;       // pin 43
            LATBbits.LATB10 = FWD;      // pin 8    
            break;
        case STLL:
            LATBbits.LATB7 = FWD;       // pin 43
            LATBbits.LATB10 = FWD;      // pin 8                  
            break;
        case IGN:               
            break;
        default:
            break;
    }
//        LATAbits.LATA4 = (LDirection == RVS)? FWD: RVS;         // pin 34
//        LATBbits.LATB4 = LDirection;                            // pin 33
//        LATBbits.LATB7 = (RDirection == RVS)? FWD: RVS;         // pin 43
//        LATBbits.LATB10 = RDirection;                           // pin 8
   
}

//****************************************************************************
// Function:    PI
//
// Description: Calculates an adjusted PWM duty cylce value based on the 
//              actual encoder value and the target encoder value.
//
// Params:      ActualEncoder - Encoder value used to calculate error
//              Motor - Left or right motor selection
//
// Return:      PWM - Adjusted PWM value
//
//****************************************************************************
uint16 PI( uint16 ActualEncoder, uint16 TrgtEncoder, uint8 Motor )
{

    float Kp = 0.4;
    float Kp1 = 0.41; 
    float Kp2 = 0.41;   
    float Ki = 0.001;     
    float dt = 0.072;
    
    sint32 error;
    float PWM;
    sint32 Integral;

    Kp = (Motor == MOTOR_1)? Kp1: Kp2;    
    error = TrgtEncoder - ActualEncoder;
    Integral = (Motor == MOTOR_1)? M1Integral: M2Integral;
    
    // add 0.5 for rounding
    PWM = (Kp * error) + (Ki * Integral);    
    
    PWM = (sint16) (( PWM < 0 )? PWM - 0.5: PWM + 0.5);
    
//    if (Motor == MOTOR_1)
//    {   
//        M1PIerror[encCnt] = error;
////        M1PI[encCnt] = PWM;
//        M1PIf[encCnt] = (Kp * error) + (Ki * Integral);
//    }
//    else
//    {
//        M2PIerror[encCnt] = error;
////        M2PI[encCnt] = PWM;
//        M2PIf[encCnt] =  (Kp * error) + (Ki * Integral); 
//    }
    
    PWM += (Motor == MOTOR_1)? OC4RS: OC3RS; 

    if (PWM > MaxPWM)
    {
        PWM = MaxPWM;        
    }
    else if (PWM < MinPWM)
    {
        PWM = MinPWM;       
    }
    else
    {
        if (TrgtEncoder == TargetEncoder)
        {
            if (Motor == 0)
            {
                M1Integral += (error * dt);   
            }
            else
            {
                M2Integral += (error * dt);      
            }    
        }
    } 
    
    return (uint16) PWM;
}

//****************************************************************************
// Function:    SetSpeed
//
// Description: 
//
// Params:      Speed - Speed selection
//
// Return:      void
//
//****************************************************************************
void SetSpeed( uint32 Speed )
{
    switch(Speed)
    {
        case(OFF):
            MotorSpeedCtrl( 0, 0 );                             
            TargetEncoder = 0;  
            Speed = OFF;
            MaxPWM = 0;
            MinPWM = 0;            
            break;
        case(SUPER_SLOW):
            MotorSpeedCtrl( SUPER_SLOW_SPEED_INIT, SUPER_SLOW_SPEED_INIT+40 );                            
            TargetEncoder = SUPER_SLOW_SPEED;
            MaxPWM = SUPER_SLOW_SPEED_INIT + 150;
            MinPWM = SUPER_SLOW_SPEED_INIT - 150;        
            SpeedCheck = SUPER_SLOW_SPEED_CHK;
            Speed = SUPER_SLOW;            
            break;            
        case(SLOW):
            MotorSpeedCtrl( SLOW_SPEED_INIT, SLOW_SPEED_INIT+35 );  
//            MotorSpeedCtrl( 0, SLOW_SPEED_INIT );  
            TargetEncoder = SLOW_SPEED;   
            MaxPWM = SLOW_SPEED_INIT + 150;
            MinPWM = SLOW_SPEED_INIT - 150;  
            SpeedCheck = SLOW_SPEED_CHK;            
            Speed = SLOW;
            break;
        case(MED):
            MotorSpeedCtrl( MED_SPEED_INIT, MED_SPEED_INIT+30 );                            
            TargetEncoder = MED_SPEED; 
            MaxPWM = MED_SPEED_INIT + 150;
            MinPWM = MED_SPEED_INIT - 150;    
            SpeedCheck = MED_SPEED_CHK;                        
            Speed = MED;            
            break;
        default:
            MotorSpeedCtrl( 0, 0 );                             
            TargetEncoder = 0;
            Speed = OFF;            
            break;            
    }
}

//****************************************************************************
// Function:    SetDirection
//
// Description: 
//
// Params:      Direction - Direction selection
//
// Return:      void
//
//****************************************************************************
void SetDirection( uint32 Direction)
{
    switch(Direction)
    {
        case(FORWARD):
                MotorDirectionCtrl( FWD, FWD );                
                TurnFlag = 0;
            break;
        case(REVERSE):
                MotorDirectionCtrl( RVS, RVS );   
                TurnFlag = 0;                
            break;  
        case(STALL_M1):
                MotorDirectionCtrl( STLL, IGN );   
            break;              
        case(STALL_M2):
                MotorDirectionCtrl( IGN, STLL );   
            break;              
        case(LEFT_90):
                MotorDirectionCtrl( FWD, RVS );  
                M1Distance = 0;
                M2Distance = 0;    
                M1PosEdgeCnt = 0;    
                M2PosEdgeCnt = 0;  
                FwdTurnDone = 0;
                RvsTurnDone = 0;
                FwdTurnCheck = MOTOR_1;                                 
                FwdTurnCnt = LEFT_TURN_FWD;
                RvsTurnCnt = LEFT_TURN_RVS;                
                TurnFlag = 1;                
            break;
        case(RIGHT_90):
                MotorDirectionCtrl( RVS, FWD ); 
                M1Distance = 0; 
                M2Distance = 0;   
                M1PosEdgeCnt = 0;   
                M2PosEdgeCnt = 0; 
                FwdTurnDone = 0;                
                RvsTurnDone = 0;
                FwdTurnCheck = MOTOR_2;                                                   
                FwdTurnCnt = RIGHT_TURN_FWD;
                RvsTurnCnt = RIGHT_TURN_RVS;                
                TurnFlag = 1;                
            break;
        case(TURN_180):
                MotorDirectionCtrl( RVS, FWD ); 
                M1Distance = 0;   
                M2Distance = 0;     
                M1PosEdgeCnt = 0;    
                M2PosEdgeCnt = 0;
                FwdTurnDone = 0;                
                RvsTurnDone = 0;
                FwdTurnCheck = MOTOR_1;                                            
                FwdTurnCnt = FULL_TURN_FWD;
                RvsTurnCnt = FULL_TURN_RVS;               
                TurnFlag = 1;               
            break;
        case(LEFT_SCAN):
                MotorDirectionCtrl( FWD, RVS );  
                M1Distance = 0;
                M2Distance = 0;    
                M1PosEdgeCnt = 0;    
                M2PosEdgeCnt = 0;  
                FwdTurnDone = 0;
                RvsTurnDone = 0;
                FwdTurnCheck = MOTOR_1;                                 
                FwdTurnCnt = SCAN_FWD;
                RvsTurnCnt = SCAN_RVS;                
                TurnFlag = 1;                          
            break;
        case(RIGHT_SCAN):
                MotorDirectionCtrl( RVS, FWD ); 
                M1Distance = 0; 
                M2Distance = 0;   
                M1PosEdgeCnt = 0;   
                M2PosEdgeCnt = 0; 
                FwdTurnDone = 0;                
                RvsTurnDone = 0;
                FwdTurnCheck = MOTOR_2;                                                   
                FwdTurnCnt = SCAN_FWD;
                RvsTurnCnt = SCAN_RVS;                
                TurnFlag = 1;            
            break;        
        default:
                MotorDirectionCtrl( FWD, FWD );
                TurnFlag = 0;         
            break;            
    }    
}

uint8 CheckFlameDetectors() 
{    
    uint8 FlameDetected = 0;
  
/*   
    i = AN10ADC; // INPUT BUFFER scaled to a 5V input
    
//    analog value 409 aprox 6 inch
    if (i<= 417 && i>=967)
    {
        //go slow 
        SetSpeed(SUPER_SLOW);
        MotorSpeedCtrl( Motor1Speed, Motor2Speed );//stop
    
   }
    // analog value 942 aprox 3 inch
    else if (i<967 && i>100)
    {
        SetSpeed(OFF);
        MotorSpeedCtrl( Motor1Speed, Motor2Speed );//stop
    
    }
*/
    
    return FlameDetected;
} 

uint8 CenterFlame()
{
   uint8 FlameCentered = 0;

//    uint8 flameDetected = 0;
//    i = 1 ; // INPUT BUFFER scaled to a 5V input
//    ADCBUFF[0]= AN10ADC;
//    ADCBUFF[1]= AN5ADC;
//    
//    if ( ADCBUFF[0] > 20 )
//    {
//        flameDetected = 1;
//    }
//    if ( ADCBUFF[1] > 20 )
//    {
//        flameDetected = 1;
//    }
//       if (cnt == 40)
//        {
//    if (flameDetected != 0)
//    {
//            LATAbits.LATA0 = 1;  
//                    
//                if (ADCBUFF[i] > ADCBUFF[i-1])
//                {
//                    RealFlame = i;
//                }
//                else /////////////////////////////
//                {
//                    RealFlame = 0;
//                }
//            if (RealFlame != 0)// center sensor
//            {
//                KeepAdjust = 0;
//                // keep adjust will turn robot
//            }
//            else 
//            {
//                KeepAdjust  = 1;
//                // stop and put out flame
//            }
//    }   
//            
//            cnt = 0;
//       }
//        
//        ADC10[cnt] = AN10ADC;
//        ADC5[cnt] = AN5ADC;
//        cnt ++;
//        
//        if (cnt > 40)
//        {
//            cnt = 0;
//        }
    
   return FlameCentered;
}

void CheckMap()
{
    
}

void CheckFrontSensor()
{
    Sens[in] = SensDiff * 0.0108844;
       
    if (SensDiff < 1063 && TurnFlag == 0)
    {
       SetDirection( RIGHT_90 );      
    }     
       
    in++;
    if (in >= 100)
    {
       in = 0;
    }   
}

void CheckCollisionSensors()
{
    if (USSensorFlag != 0)
    {
  
    }
    
    
    
}
uint8 ReRoute()
{
    uint8 AtMapLocation = 1;
    
    return AtMapLocation;
}
void ShootWater()
{
    
}

uint8 FireVerify()
{
    uint8 fireDetected = 0;
    
   
    
    
    return fireDetected;
}