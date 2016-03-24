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
    
    State = START; 
 
    GPIOInit();   
    TimerInit();
    ADCInit();
    ICInit();
    PWMInit();
    
    SetSpeed( OFF );    
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
    ANSELAbits.ANSA0 = 0;       // RA0 LED PIN    
    TRISAbits.TRISA0 = 0;       // RA0 Output   

    // ADC Pins
    ANSELAbits.ANSA1 = 1;       // RA1 Analog
    ANSELBbits.ANSB2 = 1;       // RB2 Analog
    ANSELBbits.ANSB3 = 1;       // RB3 Analog
    ANSELBbits.ANSB13 = 1;      // RB13 Analog
    ANSELBbits.ANSB14 = 1;      // RB14 Analog
//    ANSELBbits.ANSB15 = 1;      // RB15 Analog
    
    TRISAbits.TRISA1 = 1;       // RA1 Input     
    TRISBbits.TRISB2 = 1;       // RB2 Input
    TRISBbits.TRISB3 = 1;       // RB3 Input
    TRISBbits.TRISB13 = 1;      // RB13 Input
    TRISBbits.TRISB14 = 1;      // RB14 Input
//    TRISBbits.TRISB15 = 1;      // RB15 Input ///SET AS PWM INPUT SENSOR
            
    // PWM Pins
    // Pins A0, A2, A3, A4 are default 0 in ANSELA
    // Pins B4, B7, B8 are default 0 in ANSELB    
    TRISAbits.TRISA2 = 0;       //  RPA2 - OC4
    TRISAbits.TRISA3 = 0;    	//  RPA3 - OC3
    TRISAbits.TRISA4 = 0;       //  RA4 - INB1
    TRISBbits.TRISB4 = 0;		//  RB4 - INA1
    TRISBbits.TRISB7 = 0;       //  RB7 - INA2
    TRISBbits.TRISB8 = 0;		//  RB8 - INB2
    
    // IC Pins
    // Pins B5, B9 are default analog   
    ANSELBbits.ANSB15 = 0;      // RB15 Digital
    
    TRISBbits.TRISB5 = 1;       // RB5 Input
    TRISBbits.TRISB9 = 1;       // RB9 Input
    TRISBbits.TRISB15 = 1;      // RB15 Input   
    
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
    PR3 = 0x7436;               // 21Hz 47.6ms
    TMR3 = 0;                   // Reset TMR2 to 0        
    
    // Timer 3 Interrupt Config
    IEC0bits.T3IE = 0;  // TMR3 Interrupt Disable 
    IFS0bits.T3IF = 0;  // TMR3 Interrupt Flag Off
    IPC3bits.T3IP = 2;  // TMR3 Interrupt Priority
//    IPC3bits.T3IS = 1;  // TMR3 Interrupt SubPriority
    IEC0bits.T3IE = 1;  // TMR3 Interrupt Enable         
 
    T3CONbits.ON = 1;   // TMR3 Start
    
    // TIMER 4
    T4CONbits.TCKPS = 0b111;    // 1:256 prescaler
    PR4 = 0x30D4;               // 10Hz 100ms 0x3D09
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
    RPA2Rbits.RPA2R = 0b0101;   //PPS OC4 for A2 (pin 9)
    RPA3Rbits.RPA3R = 0b0101;   //PPS OC3 for A3 (pin 10)
    
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
    AD1CON2bits.SMPI = 4;       // Interrupts at the completion of conversion for each 5th sample/convert sequence
    
//    AD1CON3bits.SAMC = 31;      // Auto-Sample Time bits - 31 TAD
//    AD1CON3bits.ADCS = 1;       // ADC Conversion Clock Select bits - TPB * 2 * (ADCS + 1) = TAD
    
    AD1CHS  = 0;                // AD1 INPUT SELECT REGISTER
                                // Not needed for auto-sampling
    AD1CSSLbits.CSSL1 = 1;      // Select ANx for input scan - AN1 (pin 3)
    AD1CSSLbits.CSSL4 = 1;      // Select ANx for input scan - AN4 (pin 6)
    AD1CSSLbits.CSSL5 = 1;      // Select ANx for input scan - AN5 (pin 7)
    AD1CSSLbits.CSSL9 = 1;      // Select ANx for input scan - AN9  (pin 24)
    AD1CSSLbits.CSSL10 = 1;      // Select ANx for input scan - AN10 (pin 25)
//    AD1CSSLbits.CSSL11 = 1;      // Select ANx for input scan - AN11 (pin 26)
     
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
    // IC2
    IC2R = 4;                   // Input Pin Selection - B9 (pin 21)
    IC2CONbits.FEDGE = 1;       // Capture rising edge first
    IC2CONbits.ICTMR = 0;       // Timer 3 Select
    IC2CONbits.ICM = 0b011;     // Input Capture Mode - Simple Capture Event - Every rising edge
    IC2CONbits.ICI = 0b00;      // Interrupt on every capture event
    
    IEC0bits.IC2IE = 0;         // IC2 Interrupt Enable Off
    IFS0bits.IC2IF = 0;         // IC2 Interrupt Flag Off
    IPC2bits.IC2IP = 4;         // IC2 Interrupt Priority
//    IPC2bits.IC2IS = 2;         // IC2 Interrupt SubPriority
    
    // IC3
    IC3R = 1;                   // Input Pin Selection - B5 (pin 14)
    IC3CONbits.FEDGE = 1;       // Capture rising edge first
    IC3CONbits.ICTMR = 0;       // Timer 3 Select
    IC3CONbits.ICM = 0b011;     // Input Capture Mode - Simple Capture Event - Every rising edge
    IC3CONbits.ICI = 0b00;      // Interrupt on every capture event
    
    IEC0bits.IC3IE = 0;         // IC3 Interrupt Enable Off
    IFS0bits.IC3IF = 0;         // IC3 Interrupt Flag Off
    IPC3bits.IC3IP = 4;         // IC3 Interrupt Priority
//    IPC3bits.IC3IS = 2;         // IC3 Interrupt SubPriority

    // IC4
    IC4R = 3;                   // Input Pin Selection - B15 (pin 26)
    IC4CONbits.FEDGE = 1;       // Capture rising edge first
    IC4CONbits.ICTMR = 0;       // Timer 3 Select
    IC4CONbits.ICM = 0b110;     // Input Capture Mode - every edge, specified edge first
    IC4CONbits.ICI = 0b00;      // Interrupt on every capture event
    
    IEC0bits.IC4IE = 0;         // IC4 Interrupt Enable Off
    IFS0bits.IC4IF = 0;         // IC4 Interrupt Flag Off
    IPC4bits.IC4IP = 4;         // IC4 Interrupt Priority
//    IPC4bits.IC4IS = 4;         // IC4 Interrupt SubPriority    
    
    // ENABLE
    IEC0bits.IC2IE = 1;         // IC2 Interrupt Enable
    IEC0bits.IC3IE = 1;         // IC3 Interrupt Enable
    IEC0bits.IC4IE = 1;         // IC4 Interrupt Enable
    
    IC2CONbits.ON = 1;          // IC2 Enable    
    IC3CONbits.ON = 1;          // IC3 Enable
    IC4CONbits.ON = 1;          // IC4 Enable    
}

//****************************************************************************
// Function:    Init
//
// Description: Initializes variables and other Init functions.
//
// Params:      LSpeed - PWM Duty cycle to control left motor
//              RSpeed - PWM Duty cycle to control right motor
//              LDirection - Sets direction of left motor
//              RDirection - Sets direction of right motor
//
// Return:      void
//
//****************************************************************************
void MotorControl(uint32 M1Speed, uint32 M2Speed, uint8 LDirection, uint8 RDirection)
{
    LATAbits.LATA4 = (LDirection == 0)? 1: 0;       // pin 12
    LATBbits.LATB4 = LDirection;                    // pin 11
    LATBbits.LATB7 = (RDirection == 0)? 1: 0;       // pin 16
    LATBbits.LATB8 = RDirection;                    // pin 17
    
    OC3RS = M1Speed; // Slower motor            
    OC4RS = M2Speed;  
}

//****************************************************************************
// Function:    P
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
uint16 P( uint16 ActualEncoder, uint8 Motor)
{
//    float KpM2 = 0.5;
//    float KiM2 = 0.005;
//    float KpM1 = 0.4;
//    float KiM1 = 0.005;  
    float Kp = 0.4;
    float Ki = 0.001;     
    float dt = 0.08;
    
    sint32 error;
    sint32 val;
    uint16 PWM;
    sint32 Integral;
    
    error = TargetEncoder - ActualEncoder;
    Integral = (Motor == 0)? M1Integral: M2Integral;
//    Kp = (Motor == 0)? KpM1: KpM2;
//    Ki = (Motor == 0)? KiM1: KiM2;
    
    PWM = (Kp * error) + (Ki * Integral);
    
//    if (error >= 0) {
//        val = (error >= 20)? KP_1: (error >= 14)? KP_2: (error >= 9)? KP_3: (error >= 5)? KP_4: (error >= 3)? KP_5 : KP_6;
//    }
//    else {
//        val = (error <= -20)? -KP_1: (error <= -14)? -KP_2: (error <= -9)? -KP_3: (error <= -5)? KP_4: (error <= -3)? -KP_5 : -KP_6;
//    }
// 

    
    LastEncoderM1 = (Motor == 0)? LastEncoderM1: ActualEncoder;
    LastEncoderM2 = (Motor != 0)? LastEncoderM2: ActualEncoder;    
    
    LastErrorM1 = (Motor == 0)? LastErrorM1: error;
    LastErrorM2 = (Motor != 0)? LastErrorM2: error;
         
    LastAdjustM1 = (Motor == 0)? LastAdjustM1: val;
    LastAdjustM2 = (Motor != 0)? LastAdjustM2: val;
    
    PWM += (Motor == 0)? OC3RS: OC4RS; 
    
    if (PWM > MAX_SPEED_PWM)
    {
        PWM = MAX_SPEED_PWM;        
    }
    else if (PWM < MIN_SPEED_PWM)
    {
        PWM = MIN_SPEED_PWM;
        
    }
    else if (Motor == 0)
    {
        M1Integral += (error * dt);      
    } 
    else
    {
        M2Integral += (error * dt);      
    }     
    return PWM;
}

//****************************************************************************
// Function:    Speed
//
// Description: 
//
// Params:      Speed - Speed selection
//
// Return:      void
//
//****************************************************************************
void SetSpeed( uint32 Speed)
{
    switch(Speed)
    {
        case(OFF):
            MotorControl( 0, 0, FWRD, FWRD );                             
//            TargetEncoder = 0;
            break;
        case(SLOW):
            // 677 756
            MotorControl( SLOW_SPEED_INIT, SLOW_SPEED_INIT+80, FWRD, FWRD );                            
            TargetEncoder = SLOW_SPEED;             
            break;
        case(MED):
            MotorControl( MED_SPEED_INIT, MED_SPEED_INIT+20, FWRD, FWRD );                            
            TargetEncoder = MED_SPEED;              
            break;
        case(FAST):
            MotorControl( FAST_SPEED_INIT, FAST_SPEED_INIT+20, FWRD, FWRD );                            
            TargetEncoder = FAST_SPEED;              
            break;
        default:
            MotorControl( 0, 0, FWRD, FWRD );                             
            TargetEncoder = 0;            
            break;            
    }
}



void SensorCalc() 
{    
    i = AN10ADC * 0.66; // INPUT BUFFER scaled to a 5V input
    val=(6762/(i-9))-4;
    inch= val/2.54;
    
    
//    if (inch < 24)
//    {
        if (cnt == 40)
        {
//            LATAbits.LATA0 = 1;         
//            DebugFlag();   
            cnt = 0;
        }
        Inches[cnt] = inch;
        ADC10[cnt] = AN10ADC;
        cnt ++;
//    }
}  

void DebugFlag()
{
    SetSpeed(OFF);   

    LATAbits.LATA0 = 0;  
}