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
    MapInit();
    SetSpeed( OFF );    
    SetDirection( FORWARD );      
    MotorSpeedCtrl( Motor1Speed, Motor2Speed );   
    
    M1Distance = 0;
    M2Distance = 0;
    
    Extinguish = 0;
    FollowingMap = 1;
    MapIndex = 0;

    M1_SSlow = SUPER_SLOW_SPEED_INIT;
    M2_SSlow = SUPER_SLOW_SPEED_INIT + 40;    
    M1_Slow = SLOW_SPEED_INIT;
    M2_Slow = SLOW_SPEED_INIT + 40;
    M1_Med = MED_SPEED_INIT;
    M2_Med = MED_SPEED_INIT + 30;
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
    ANSELB = 0;

    TRISAbits.TRISA7 = 0;       // RA7 Output   // 
    TRISBbits.TRISB9 = 0;       // RB9 Output   // 
    TRISAbits.TRISA10 = 0;      // RA10 Output  // 
    TRISCbits.TRISC7 = 0;       // RC7 Output   // Motor Dist main
    TRISCbits.TRISC8 = 0;       // RC8 Output   // 

    // Buttons
    //1 0000 1000
    ANSELA &= ~(0x0108);
    TRISAbits.TRISA3 = 1;			// Butt on RA3
    TRISAbits.TRISA8 = 1;			// Butt on RA8    
    
    // ADC Pins
    // Flame Sensors
    ANSELAbits.ANSA0    = 1;       // RA0 Analog  - AN0
    ANSELAbits.ANSA1    = 1;       // RA1 Analog  - AN1
    ANSELCbits.ANSC3   = 1;       // RC3 Analog - AN12
    ANSELCbits.ANSC0    = 1;       // RC0 Analog  - AN6
    ANSELCbits.ANSC1    = 1;       // RC1 Analog  - AN7    
    TRISAbits.TRISA0    = 1;       // RA0 Input  - AN0        
    TRISAbits.TRISA1    = 1;       // RA1 Input  - AN1     
    TRISCbits.TRISC3   = 1;       // RC3 Input  - AN12
    TRISCbits.TRISC0    = 1;       // RC0 Input  - AN6
    TRISCbits.TRISC1    = 1;       // RC1 Input  - AN7 

    // IR Sensors
    ANSELBbits.ANSB13   = 1;      // RB13 Analog - AN11
    ANSELBbits.ANSB14   = 1;      // RB14 Analog - AN10
    ANSELBbits.ANSB15   = 1;      // RB15 Analog - AN9
    ANSELCbits.ANSC2    = 1;      // RC3 Analog  - AN8
    TRISBbits.TRISB13   = 1;      // RB13 Input - AN11
    TRISBbits.TRISB14   = 1;      // RB14 Input - AN10
    TRISBbits.TRISB15   = 1;      // RB15 Input - AN9            
    TRISCbits.TRISC2    = 1;      // RC3 Input  - AN8 
    
    // PWM Pins
    TRISAbits.TRISA2 = 0;       //  RPA2 OC4    - PWM2
    TRISBbits.TRISB11 = 0;    	//  RPB11 OC2   - PWM1
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
// Description: Initializes OC2 and OC4 as PWM.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void PWMInit(void)
{
    RPA2Rbits.RPA2R = 0b0101;   //PPS OC4 for A2 (pin 30)
    RPB11Rbits.RPB11R = 0b0101;   //PPS OC2 for B11 (pin 9)
    
    OC2CON = 0x0000;            // OC2 Disable
    OC2CON = 0x0006;            // OCM PWM mode   
    OC2RS = 0;             // OC2 Secondary Compare Register
                                // 0x1F4 0x4E2 0x659 0X6D6 0x753 0x7D0
                                // 0x7FF //0x698 //0x392 //0x3FF 
    OC4CON = 0x0000;            // OC4 Disable
    OC4CON = 0x0006;            // OCM PWM mode   
    OC4RS = 0;             // OC4 Secondary Compare Register
  
    OC2CONSET = 0x8000;         // OC2 Enable
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
    AD1CON1bits.SSRC = 0b111;   // Auto Convert
    AD1CON1bits.CLRASAM = 1;    // Stop conversions when the first ADC interrupt is generated
                                // Will also clear ASAM bit  
    AD1CON2bits.CSCNA = 1;      // Scan Inputs
    AD1CON2bits.SMPI = 8;       // Interrupts at the completion of conversion for each 9th sample/convert sequence
    
    AD1CON3bits.SAMC = 26;      // Auto-Sample Time bits - 31 TAD
    AD1CON3bits.ADCS = 4;       // ADC Conversion Clock Select bits - TPB * 2 * (ADCS + 1) = TAD
    
    AD1CHS  = 0;                // AD1 INPUT SELECT REGISTER
                                // Not needed for auto-sampling
    AD1CSSLbits.CSSL0 = 1;      // Select ANx for input scan - AN0 (pin 19) 
    AD1CSSLbits.CSSL1 = 1;      // Select ANx for input scan - AN1 (pin 20)
    AD1CSSLbits.CSSL6 = 1;      // Select ANx for input scan - AN6 (pin 25)
    AD1CSSLbits.CSSL7 = 1;      // Select ANx for input scan - AN7 (pin 26)
    AD1CSSLbits.CSSL8 = 1;      // Select ANx for input scan - AN8 (pin 27)
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

void MapInit()
{
    
      //Test     
//    MapDistance[0] = 8;
//    MapDirection[0] = FORWARD;    
//    MapDistance[1] = 0;
//    MapDirection[1] = RIGHT_45;  // 45 degree angle ish  
//    MapDistance[2] = 1;
//    MapDirection[2] = FORWARD;     
//    MapDistance[3] = 0;
//    MapDirection[3] = LEFT_45;  // 45 degree angle ish     
//    MapDistance[4] = 1;
//    MapDirection[4] = FORWARD;  // Room 1 Door
//    MapDistance[5] = 0;
//    MapDirection[5] = FORWARD; 
//    MapDistance[6] = 1;
//    MapDirection[6] = FORWARD;     
//    MapDistance[7] = 0;
//    MapDirection[7] = FORWARD;         
    /////////
    
    MapDistance[0] = 35; //350; 
    MapDirection[0] = FORWARD;    
    MapDistance[1] = 1;
    MapDirection[1] = FORWARD;  // 45 degree angle ish - adjust RIGHT_45_TURN_RVS and RIGHT_45_TURN_FWD
    MapDistance[2] = 17; //180;
    MapDirection[2] = FORWARD;     
    MapDistance[3] = 1;
    MapDirection[3] = FORWARD;  // 45 degree angle ish  - adjust LEFT_45_TURN_RVS and LEFT_45_TURN_FWD   
    MapDistance[4] = 13;
    MapDirection[4] = FORWARD;  // check left sensors for Room 1 Door 
    MapDistance[5] = 0;
    MapDirection[5] = LEFT_45; 
    MapDistance[6] = 15;
    MapDirection[6] = FORWARD; 
    MapDistance[7] = 5;
    MapDirection[7] = FORWARD; 
    MapDistance[8] = 5;
    MapDirection[8] = FORWARD; 
    MapDistance[9] = 0;
    MapDirection[9] = LEFT_45;     
    MapDistance[10] = 35;
    MapDirection[10] = FORWARD;  
    MapDistance[11] = 0;
    MapDirection[11] = RIGHT_90; 
    MapDistance[12] = 29;
    MapDirection[12] = FORWARD;  
    MapDistance[13] = 0;
    MapDirection[13] = RIGHT_90;     
    MapDistance[14] = 25;
    MapDirection[14] = FORWARD; 
    MapDistance[15] = 0;     
    MapDirection[15] = RIGHT_90; 
    MapDistance[16] = 45;
    MapDirection[16] = FORWARD; 
    MapDistance[17] = 0;
    MapDirection[17] = LEFT_90;          
    MapDistance[18] = 24;
    MapDirection[18] = FORWARD; // Room 1 Exit    
    MapDistance[19] = 0;
    MapDirection[19] = LEFT_90;  
    MapDistance[20] = 63;
    MapDirection[20] = FORWARD; // check left sensors for  Room 2 Door    
    MapDistance[21] = 0;
    MapDirection[21] = LEFT_90; 
    MapDistance[22] = 24;
    MapDirection[22] = FORWARD; 
    MapDistance[23] = 0;
    MapDirection[23] = RIGHT_90;     
    MapDistance[24] = 10;
    MapDirection[24] = FORWARD; 
    MapDistance[25] = 0;
    MapDirection[25] = LEFT_90;  
    MapDistance[26] = 36;
    MapDirection[26] = FORWARD;  
    MapDistance[27] = 0;
    MapDirection[27] = RIGHT_90; 
    MapDistance[28] = 40;
    MapDirection[28] = FORWARD;  
    MapDistance[29] = 0;
    MapDirection[29] = RIGHT_90;     
    MapDistance[30] = 36;
    MapDirection[30] = FORWARD; 
    MapDistance[31] = 0;   
    MapDirection[31] = RIGHT_90; 
    MapDistance[32] = 50;
    MapDirection[32] = FORWARD; 
    MapDistance[33] = 0;
    MapDirection[33] = LEFT_90;      
    MapDistance[34] = 24;
    MapDirection[34] = FORWARD; // Room 2 Exit     
    MapDistance[35] = 0;
    MapDirection[35] = LEFT_90;  
    MapDistance[36] = 66;
    MapDirection[36] = FORWARD; // check left sensors for Room 3 Door     
    MapDistance[37] = 0;
    MapDirection[37] = LEFT_90; 
    MapDistance[38] = 24;
    MapDirection[38] = FORWARD; 
    MapDistance[39] = 0;
    MapDirection[39] = RIGHT_90;    
    MapDistance[40] = 10;
    MapDirection[40] = FORWARD; 
    MapDistance[41] = 0;
    MapDirection[41] = LEFT_90;      
    MapDistance[42] = 36;
    MapDirection[42] = FORWARD;  
    MapDistance[43] = 0;
    MapDirection[43] = RIGHT_90; 
    MapDistance[44] = 40;
    MapDirection[44] = FORWARD;  
    MapDistance[45] = 0;
    MapDirection[45] = RIGHT_90;     
    MapDistance[46] = 36;
    MapDirection[46] = FORWARD;     
    MapDistance[47] = 0;         
    MapDirection[47] = RIGHT_90; 
    MapDistance[48] = 50;
    MapDirection[48] = FORWARD; 
    MapDistance[49] = 0;
    MapDirection[49] = LEFT_90;      
    MapDistance[50] = 24;
    MapDirection[50] = FORWARD; // Room 3 Exit         
    MapDistance[51] = 0;
    MapDirection[51] = LEFT_90;  
    MapDistance[52] = 50;
    MapDirection[52] = FORWARD;   
    MapDistance[53] = 0;
    MapDirection[53] = TURN_180; 
    MapDistance[54] = 50;
    MapDirection[54] = FORWARD; // check left sensors for Room 3 Door    
    MapDistance[55] = 660;
    MapDirection[55] = FORWARD; // check left sensors for Room 2 Door 
    MapDistance[56] = 63;
    MapDirection[56] = FORWARD; // check left sensors for Room 1 Door 
    MapDistance[57] = 13;
    MapDirection[57] = FORWARD;       
    MapDistance[58] = 0;
    MapDirection[58] = RIGHT_45;  // 45 degree angle ish  
    MapDistance[59] = 18;
    MapDirection[59] = FORWARD;     
    MapDistance[60] = 0;
    MapDirection[60] = LEFT_45;  // 45 degree angle ish      
    MapDistance[61] = 35;
    MapDirection[61] = FORWARD; 
    MapDistance[62] = 0;
    MapDirection[62] = FORWARD;          
    
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
    OC4RS = M1Speed; // Slower motor            
    OC2RS = M2Speed;      
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
            LATAbits.LATA4 = FWD;       
            LATBbits.LATB4 = RVS;                      
            break;
        case FWD:
            LATAbits.LATA4 = RVS;   
            LATBbits.LATB4 = FWD;          
            break;
        case STLL:
            LATAbits.LATA4 = FWD;     
            LATBbits.LATB4 = FWD;                   
            break;
        case IGN:               
            break;
        default:
            break;
    }        
 
    switch (RDirection)
    {
        case RVS:
            LATBbits.LATB7 = FWD;    
            LATBbits.LATB10 = RVS;                   
            break;
        case FWD:
            LATBbits.LATB7 = RVS;       
            LATBbits.LATB10 = FWD;     
            break;
        case STLL:
            LATBbits.LATB7 = FWD;     
            LATBbits.LATB10 = FWD;                    
            break;
        case IGN:               
            break;
        default:
            break;
    }                 
   
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
    
    // Testing stuff
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
    ///////
    
    PWM += (Motor == MOTOR_1)? OC4RS: OC2RS; 

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
void SetSpeed( uint32 Spd )
{
    AdjustSpeedFlag = 0;
    switch(Spd)
    {
        case(OFF):
            MotorSpeedCtrl( 0, 0 );                             
            TargetEncoder = 0;  
            CurrSpeed = OFF;
            MaxPWM = 0;
            MinPWM = 0;            
            break;
        case(SUPER_SLOW):
            MotorSpeedCtrl( M1_SSlow, M2_SSlow );                            
            TargetEncoder = SUPER_SLOW_SPEED;
            MaxPWM = M1_SSlow + 150;
            MinPWM = M1_SSlow - 150;        
            SpeedCheck = SUPER_SLOW_SPEED_CHK;
            CurrSpeed = SUPER_SLOW;            
            break;            
        case(SLOW):
            MotorSpeedCtrl( M1_Slow, M2_Slow );  
//            MotorSpeedCtrl( 0, SLOW_SPEED_INIT );  
            TargetEncoder = SLOW_SPEED;   
            MaxPWM = M1_Slow + 150;
            MinPWM = M1_Slow - 150;  
            SpeedCheck = SLOW_SPEED_CHK;            
            CurrSpeed = SLOW;
            break;
        case(MED):
            MotorSpeedCtrl( M1_Med, M2_Med );                            
            TargetEncoder = MED_SPEED; 
            MaxPWM = M1_Med + 150;
            MinPWM = M1_Med - 150;    
            SpeedCheck = MED_SPEED_CHK;                        
            CurrSpeed = MED;            
            break;
        default:
            MotorSpeedCtrl( 0, 0 );                             
            TargetEncoder = 0;
            CurrSpeed = OFF;            
            break;            
    }
}

void SaveCurrEnc( void )
{
    switch (CurrSpeed)
    {
        case(SUPER_SLOW):
            M1_SSlow = OC4RS;
            M2_SSlow = OC2RS;
            break;
        case(SLOW):
            M1_Slow = OC4RS;
            M2_Slow = OC2RS;            
            break;            
        case(MED):
            M1_Med = OC4RS;
            M2_Med = OC2RS;            
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
//              can be simplified
//****************************************************************************
void SetDirection( uint32 Direction )
{
    uint8 TurnVar = 0;
    
    switch(Direction)
    {
        case(FORWARD):
                MotorDir = FORWARD;
                MotorDirectionCtrl( FWD, FWD );                
                TurnFlag = 0;
            break;
        case(REVERSE):
                MotorDir = REVERSE;
                MotorDirectionCtrl( RVS, RVS );   
                TurnFlag = 0;                
            break;  
        case(STALL_M1):      
                MotorDirectionCtrl( STLL, IGN );   
            break;              
        case(STALL_M2):       
                MotorDirectionCtrl( IGN, STLL );   
            break;              
        case(LEFT_45):
                MotorDirectionCtrl( FWD, RVS );  

                FwdTurnCheck = MOTOR_1;                                 
                FwdTurnCnt = TURN_45_ENC;
                RvsTurnCnt = TURN_45_ENC;                
                TurnFlag = 1;
                TurnVar = 1;                
            break;
        case(RIGHT_45):
                MotorDirectionCtrl( RVS, FWD ); 

                FwdTurnCheck = MOTOR_2;                                                   
                FwdTurnCnt = TURN_45_ENC;
                RvsTurnCnt = TURN_45_ENC;                
                TurnFlag = 1;                
                TurnVar = 1;                
            break;
        case(LEFT_90):
                MotorDirectionCtrl( FWD, RVS );  

                FwdTurnCheck = MOTOR_1;                                 
                FwdTurnCnt = TURN_90_ENC;
                RvsTurnCnt = TURN_90_ENC;                
                TurnFlag = 1;                
                TurnVar = 1;                
            break;
        case(RIGHT_90):
                MotorDirectionCtrl( RVS, FWD ); 

                FwdTurnCheck = MOTOR_2;                                                   
                FwdTurnCnt = TURN_90_ENC;
                RvsTurnCnt = TURN_90_ENC;                
                TurnFlag = 1;                
                TurnVar = 1;                
            break;
        case(TURN_180):
                MotorDirectionCtrl( RVS, FWD ); 

                FwdTurnCheck = MOTOR_2;                                            
                FwdTurnCnt = TURN_180_ENC;
                RvsTurnCnt = TURN_180_ENC;               
                TurnFlag = 1;               
                TurnVar = 1;                
            break;
        case(LEFT_SCAN):
                MotorDirectionCtrl( FWD, RVS );  

                FwdTurnCheck = MOTOR_1;                                 
                FwdTurnCnt = TURN_SCAN_ENC;
                RvsTurnCnt = TURN_SCAN_ENC;                
                TurnFlag = 1;                          
                TurnVar = 1;                
            break;
        case(RIGHT_SCAN):
                MotorDirectionCtrl( RVS, FWD ); 

                FwdTurnCheck = MOTOR_2;                                                   
                FwdTurnCnt = TURN_SCAN_ENC;
                RvsTurnCnt = TURN_SCAN_ENC;                
                TurnFlag = 1;            
                TurnVar = 1;                
            break;        
        default:
                MotorDirectionCtrl( FWD, FWD );
                TurnFlag = 0;         
            break;            
    }    
    
    if ( TurnVar != 0 )
    {
//        M2_SSlow = ( FwdTurnCheck == MOTOR_1 )? M2_SSlow : M2_SSlow-100;       
//        M2_Slow = ( FwdTurnCheck == MOTOR_1 )? M2_Slow : M2_Slow-100;       
//        M2_Med = ( FwdTurnCheck == MOTOR_1 )? M2_Med : M2_Med-100;       
//        M1_SSlow = ( FwdTurnCheck != MOTOR_1 )? M1_SSlow : M1_SSlow-100;       
//        M1_Slow = ( FwdTurnCheck != MOTOR_1 )? M1_Slow : M1_Slow-100;       
//        M1_Med = ( FwdTurnCheck != MOTOR_1 )? M1_Med : M1_Med-100;             
        
        M1Distance = 0;
        M2Distance = 0;    
        M1PosEdgeCnt = 0;    
        M2PosEdgeCnt = 0;  
        FwdTurnDone = 0;
        RvsTurnDone = 0;        
         
        TurnFlag = 1;                            
    }        
       
}

uint8 CheckFlameDetectors() 
{    
    uint8 FlameDetected = 0;
    uint32 i; 
 
    // test with brpken sensors
//    FlameSens[0] = 0;
//    FlameSens[1] = 0;                
//    FlameSens[2] = 0;    
    
    for (i = 1; i < 5; i++)
    {       
        if (FlameSens[i] > 14)
        {
            FlameDetected = 1;
        }            
    }
  
    return FlameDetected;
} 

uint32 CenterFlame()
{
   uint32 ActualCenter = 0;
   uint32 i; 
   uint16 MaxHighest = 0;

   // test with broken sensors
//    FlameSens[0] = 0;
//    FlameSens[1] = 0;                
//    FlameSens[2] = 0;   
   
    for (i = 0; i < 5; i++)
    {          
        if (MaxHighest < FlameSens[i])
        {
            MaxHighest = FlameSens[i];
            ActualCenter = i;
            
        }            
    }

   return ActualCenter;
}

void CheckMap()
{    
    if ( MapDist == MapDistance[MapIndex] && TurnFlag == 0 )
    {
        MapDist = 0;
        M1Distance = 0;
        M2Distance = 0;   
        
        MapIndex++;
        dir = MapDirection[MapIndex];
        
//        if ( MapIndex == 0 )
//        {
//            WallFollowing  = 1;
//        } 
//        else
//        {
//            WallFollowing  = 0; 
//        }
            
        if ( MapIndex < MAP_MAX )
        {
            if ( MapDirection[MapIndex] != FORWARD &&  MapDirection[MapIndex] != REVERSE && MapDirection[MapIndex] != DIR_OFF )
            {
                SaveCurrEnc();               
                SetDirection( MapDirection[MapIndex] );
                NextDir = MapDirection[MapIndex+1];  
                NextSpeed = SLOW;       
//                WallFollowing = 0;
            }
            else 
            {
                SetSpeed( SLOW );                  
                SetDirection( MapDirection[MapIndex] );                  
                
            }
        }
        else
        {
            SetDirection( STALL_M1 );
            SetDirection( STALL_M2 );
            SetSpeed( OFF );           
//            WallFollowing = 0;
            MapDone = 1;
        }      
    }
}

void CheckFrontSensor()
{
    uint32 TurnDir; 
       
    if (USSensDiff < 1063 && TurnFlag == 0)
    {
       TurnDir = CheckCollisionSensors();
//       if ( TurnDir == NO_COLLISION )
//       {
//           TurnDir = LEFT_90;
//       }
//       SetSpeed( SUPER_SLOW );
       
       SetDirection( TurnDir ); 
       NextDir = FORWARD;
       NextSpeed = CurrSpeed;

       UnMappedTurn++;
    }     
    
    // testing
//    USSens[USInd] = USSensDiff * 0.0108844;    
//    USInd++;
//    if (USInd >= 100)
//    {
//       USInd = 0;
//    }   
}

uint32 CheckCollisionSensors()
{
    uint32 Collision;
    Collision = LEFT_90;
    
    if ( (IRSens[0] > 750) || (IRSens[1] > 750) ) // Left Sensors
    {
        Collision = RIGHT_90;
    }
    if ( (IRSens[2] > 750) && (IRSens[3] > 750) ) // Right Sensors
    {
        Collision = (Collision == RIGHT_90)? TURN_180: LEFT_90;
//        if (Collision == RIGHT_90)
//        {
//            Collision = TURN_180; // or reverse
//        }
//        else
//        {
//            Collision = LEFT_90;
//        }
    }   
     
    return Collision;
}
uint8 ReRoute()
{
    uint8 AtMapLocation = 1;
    
    ////// need to work on
    ////// need an array
    
    return AtMapLocation;
}
uint8 ShootWater()
{
    uint8 plseDne = 0;
    
    // 100 ms pulse    
    if ( WaterPulse == 0 )
    {
        LATCbits.LATC4 = 0;        
        plseDne = 1;
    }
    else
    {
        LATCbits.LATC4 = 1;
    }
    return plseDne;
}

uint8 FireVerifyTemp()
{
    uint8 fireDetected = 0;
//    float temp = 0;
    uint8 s1 = 0;
    uint8 s2 = 0;
    uint8 s3 = 0;

    // add functions from other project
    I2C2Start();  
    if (I2C2Send((MLX90614_I2CADDR<<1) & 0xFE) == 0)     
    {
        s1 = 1;
        if ( I2C2Send(MLX90614_TOBJ & 0xFF) == 0 )
        {
            s2 = 1;
            I2C2Restart();      
            if ( I2C2Send((MLX90614_I2CADDR<<1) | 0x01) == 0 )    
            {
                tmpta = readTemp();  
            
//        a++;    
   
                I2C2Stop(); 
                s3 = 1;
                
            }
        }
    }
    if (tmpta > 79)
    {
        fireDetected = 1;
    }
    
    return fireDetected;
}

uint8 FireVerifySens()
{
    uint8 fireDetected = 1;
    sint32 flSensDif;
    flSensDif = flmMidMax - flmMidMin;
    
    if (flSensDif > 20)
    {
       fireDetected = 0; 
    }
    
    return fireDetected;    
}

uint8 CheckWalls()
{
    uint8 AdjustWall = 1;
    uint16 RBTemp;
    uint16 RFTemp;
    uint16 thrsh;
    uint16 nthrsh;
    
    thrsh = 8;
    nthrsh = 8;    
    RBTemp = IRSens[2];
    RFTemp = IRSens[3];
    // thrsh    clsr
    // 500      400 pos val m1 inc
    // thrsh    frth
    // 300      400 neg val m1 dec
    
    RBChng =  RBThresh - RBTemp;        
    RFChng =  RFThresh - RFTemp;        
   
    if (RFChng > thrsh)
    {
        RFState = INC;     
        M2Wall = RFChng/thrsh;        
    }
    else if (RFChng < -thrsh)
    {
        RFState = DEC; 
        M2Wall = -(RFChng/thrsh);       
    }    
    else
    {
        AdjustWall = 0;        
        RFState = STEADY;   
        M2Wall = 0;
    }       
    
    if (RBChng > nthrsh)
    {
        RBState = INC; 
        M1Wall = RFChng/nthrsh; 
        if (RFState == INC)
        {
            M1Wall = -M1Wall;              
        }
    }
    else if (RBChng < -nthrsh)
    {
        RBState = DEC;  
        M1Wall = -(RFChng/nthrsh);  
        if (RFState == DEC)
        {
            M1Wall = -M1Wall;              
        }        
    }    
    else
    {
        AdjustWall = 0;        
        RBState = STEADY;  
        M1Wall = 0;        
    }
        
    return AdjustWall;    
}

uint8 DecoyCheck()
{
    uint8 ChkCmplt = 0;
    
//    FlLM[flcn] = FlameSens[3];            
//    FlM[flcn] = FlameSens[2];
//    FlRM[flcn] = FlameSens[1];            
//                    
//                    
//    flcn ++;
//    if (flcn >= 2025)
//    {                    
//        flcn = 0;
//    }
                    
    if (IgnFirst == 0)
    {
        flMidDif = FlameSens[2] - PrvMidFlame;
        flLftDif = FlameSens[3] - PrvLftFlame;
        flRgtDif = FlameSens[1] - PrvRgtFlame;

//    if (flMidDif > 20 || flMidDif < -20)
//    {
//       flcn = 0;
//    }


    //   FlRM[flcnt] = FlameSens[1]; 
    //   FlR[flcnt] = FlameSens[0]; 
    flmMidMax  = ( flMidDif>flmMidMax )? flMidDif: flmMidMax;
    flmMidMin  = ( flMidDif<flmMidMin )? flMidDif: flmMidMin;
                        
    flmLftMax  = ( flLftDif>flmLftMax )? flLftDif: flmLftMax;
    flmLftMin  = ( flLftDif<flmLftMin )? flLftDif: flmLftMin;

    flmRgtMax  = ( flRgtDif>flmRgtMax )? flRgtDif: flmRgtMax;
    flmRgtMin  = ( flRgtDif<flmRgtMin )? flRgtDif: flmRgtMin;                        
    }
    else 
    {
        IgnFirst = 0; 
    }
    PrvMidFlame = FlameSens[2];  
    PrvLftFlame = FlameSens[3];                    
    PrvRgtFlame = FlameSens[1];                    
                    
    flcnt++;
    if (flcnt >= 30381)
    {
        
        flcnt = 0;
        ChkCmplt = 1;
        //debug reset
//        flmMidMin = 32767;
//        flmMidMax = -32768; 
//        flmLftMin = 32767;
//        fmLftMax = -32768;                          
//        flmRgtMin = 32767;
//        flmRgtMax = -32768;                          
//        IgnFirst = 1;
    }    
    
    
    return ChkCmplt;
}

uint8 CheckForDoor(uint8 Side)
{
    uint8 DoorCheck = CLEAR_SENS;
    uint16 BackSens;
    uint16 FrontSens;
    
    BackSens = (Side == LEFT_SIDE)? IRSens[0]: IRSens[2];
    FrontSens = (Side == LEFT_SIDE)? IRSens[1]: IRSens[3];
    
    if ( FrontSens > 50 )
    {
        DoorCheck = FRONT_SENS;
    }
    if ( BackSens > 50 )
    {
        DoorCheck = (DoorCheck == FRONT_SENS)? BOTH_SENS: BACK_SENS;        
    }    
    return DoorCheck;
}


///I2C2
float readTemp()
{
  float tempCalc = 0;
  
  I2C2CONbits.ACKDT = 0;    
  tempBuffLo = I2C2Read();
  
  I2C2CONbits.ACKDT = 0;  
  tempBuffHi = I2C2Read();
  
  I2C2CONbits.ACKDT = 1;
  CRCBuff = I2C2Read();

  
  temp16 = (tempBuffHi << 8) | tempBuffLo;
  tempCalc = temp16 * 0.02;
  tempCalc  -= 273.15;    
  tempCalc = tempCalc * 9.0 / 5.0 + 32;
  return tempCalc;
}

void I2C2Init(int BRG)
{
    IPC9bits.I2C2IP = 3;
    IFS1bits.I2C2MIF = 0;
    IFS1bits.I2C2BIF = 0;  
    IFS1bits.I2C2SIF = 0;  
    IEC1bits.I2C2MIE = 1;      
    IEC1bits.I2C2BIE = 1;  
    IEC1bits.I2C2SIE = 1;  
    
    I2C2BRG = BRG;
    I2C2CONbits.DISSLW = 1;    
    I2C2CONbits.I2CEN = 1;

    cnt = 1;
    while ( cnt <= 16000 );
    cnt = 0;
    
    I2C2Stop();
}

void I2C2Stop()
{
	I2C2CONbits.PEN = 1;         /* Stop condition enabled */
	while(I2C2CONbits.PEN);      /* Wait for stop condition to finish */
    BusColision();
    I2C2CONbits.RCEN = 0;
    
//    cnt = 1;
//    while ( cnt <= 500 );
//    cnt = 0;
}

void I2C2Start()
{
    I2C2CONbits.ACKDT = 0;
    while(I2C2STATbits.P == 0);
	I2C2CONbits.SEN = 1;         /* Start condition enabled */
	while(I2C2CONbits.SEN);      /* automatically cleared by hardware */
    
    BusColision();               /* wait for start condition to finish */
}

void I2C2Restart()
{
    I2C2Idle();
	I2C2CONbits.RSEN = 1;        /* Repeated start enabled */
	while(I2C2CONbits.RSEN);     /* wait for condition to finish */
    BusColision();
}

uint8 I2C2Send(uint8 addr)
{	      
    while(I2C2STATbits.S == 0);
    
    I2C2TRN = addr;  
    while(I2C2STATbits.TBF != 0); 
    while (I2C2STATbits.TRSTAT != 0);
    BusColision();
	
    if(I2C2STATbits.ACKSTAT != 0)
    {             
//        acks++;
//    }
//    else
//    {
        I2C2Stop();
        return 1;
//        
//        nacks++;
        
    }
    return 0;
}

uint8 I2C2Read(void)
{
	int temp;
    I2C2Idle();
	I2C2CONbits.RCEN = 1;         
	while(I2C2CONbits.RCEN != 0);   
    
    I2C2Idle();
    BusColision();
	temp = I2C2RCV;   
    I2C2CONbits.ACKEN = 1;
	while(I2C2CONbits.ACKEN != 0);   
   
	return temp;                  
}

void I2C2Idle(void)
{
    while ((I2C2CONbits.ACKEN != 0) || (I2C2CONbits.RCEN != 0) 
            || (I2C2CONbits.PEN != 0) || (I2C2CONbits.RSEN != 0) 
            || (I2C2CONbits.SEN != 0) );
    
}
 void BusColision(void)
 {
 	if(I2C2STATbits.BCL || I2C2STATbits.IWCOL)
     {         
        I2C2STATbits.BCL = 0;
        I2C2STATbits.IWCOL = 0;
     }
 }
