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
void Init( void )
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
void GPIOInit( void )
{
    // LED Pins
    ANSELB = 0;

    TRISAbits.TRISA7    = 0;    // RA7 Output    Left LED
    TRISBbits.TRISB9    = 0;    // RB9 Output    Right LED 
    TRISAbits.TRISA10   = 0;    // RA10 Output   Top LED
    TRISCbits.TRISC7    = 0;    // RC7 Output    PCB LED
    TRISCbits.TRISC8    = 0;    // RC8 Output    Bottom LED

    // Buttons
    ANSELA &= ~(0x0108);        // Clear ANSELA bits
    TRISAbits.TRISA3    = 1;	// Butt on RA3   Battle Button
    TRISAbits.TRISA8    = 1;	// Butt on RA8   Start Button 
    
    // ADC Pins
    // Flame Sensors
    ANSELAbits.ANSA0    = 1;    // RA0 Analog   - AN0
    ANSELAbits.ANSA1    = 1;    // RA1 Analog   - AN1
    ANSELCbits.ANSC3    = 1;    // RC3 Analog   - AN12
    ANSELCbits.ANSC0    = 1;    // RC0 Analog   - AN6
    ANSELCbits.ANSC1    = 1;    // RC1 Analog   - AN7    
    TRISAbits.TRISA0    = 1;    // RA0 Input    - AN0        
    TRISAbits.TRISA1    = 1;    // RA1 Input    - AN1     
    TRISCbits.TRISC3    = 1;    // RC3 Input    - AN12
    TRISCbits.TRISC0    = 1;    // RC0 Input    - AN6
    TRISCbits.TRISC1    = 1;    // RC1 Input    - AN7 

    // IR Sensors
    ANSELBbits.ANSB13   = 1;    // RB13 Analog  - AN11
    ANSELBbits.ANSB14   = 1;    // RB14 Analog  - AN10
    ANSELBbits.ANSB15   = 1;    // RB15 Analog  - AN9
    ANSELCbits.ANSC2    = 1;    // RC3 Analog   - AN8
    TRISBbits.TRISB13   = 1;    // RB13 Input   - AN11
    TRISBbits.TRISB14   = 1;    // RB14 Input   - AN10
    TRISBbits.TRISB15   = 1;    // RB15 Input   - AN9            
    TRISCbits.TRISC2    = 1;    // RC3 Input    - AN8 
    
    // PWM Pins
    TRISAbits.TRISA2    = 0;    // RPA2     OC4 - PWM2
    TRISBbits.TRISB11   = 0;    // RPB11    OC2 - PWM1
    TRISAbits.TRISA4    = 0;    // RA4          - INB1
    TRISBbits.TRISB4    = 0;	// RB4          - INA1
    TRISBbits.TRISB7    = 0;    // RB7          - INA2
    TRISBbits.TRISB10   = 0;	// RB10         - INB2
    
    // IC Pins
    TRISBbits.TRISB5    = 1;    // RB5 Input
    TRISCbits.TRISC5    = 1;    // RC5 Input
    TRISCbits.TRISC6    = 1;    // RC6 Input   
    
    // Solenoid
    TRISCbits.TRISC4    = 0;    //  RPC4 Output
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
void TimerInit( void )
{
    // TIMER 2
    PR2 = 0x09C4;               // 16KHz 62.5us
    TMR2 = 0;                   // Reset TMR2 to 0        
    
    IEC0bits.T2IE = 0;          // TMR2 Interrupt Disable 
    IFS0bits.T2IF = 0;          // TMR2 Interrupt Flag Off
    IPC2bits.T2IP = 2;          // TMR2 Interrupt Priority
 
    // TIMER 3
    T3CONbits.TCKPS = 0b110;    // 1:64 prescaler
    PR3 = 0x77A1;               // 20.408Hz 49ms
    TMR3 = 0;                   // Reset TMR3 to 0        
    
    IEC0bits.T3IE = 0;          // TMR3 Interrupt Disable 
    IFS0bits.T3IF = 0;          // TMR3 Interrupt Flag Off
    IPC3bits.T3IP = 2;          // TMR3 Interrupt Priority
 
    // TIMER 4
    T4CONbits.TCKPS = 0b111;    // 1:256 prescaler
    PR4 = 0x2BF2;               // 13.89Hz 72ms    
    TMR4 = 0;                   // Reset TMR4 to 0 
    
    IEC0bits.T4IE = 0;          // TMR4 Interrupt Disable 
    IFS0bits.T4IF = 0;          // TMR4 Interrupt Flag Off
    IPC4bits.T4IP = 2;          // TMR4 Interrupt Priority

    IEC0bits.T2IE = 1;          // TMR2 Interrupt Enable             
    IEC0bits.T3IE = 1;          // TMR3 Interrupt Enable          
    IEC0bits.T4IE = 1;          // TMR4 Interrupt Enable    
 
    T2CON = 0x8000;             // TMR2 Start    
    T3CONbits.ON  = 1;          // TMR3 Start
    T4CONbits.ON  = 1;          // TMR4 Start
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
void PWMInit( void )
{
    RPA2Rbits.RPA2R = 0b0101;   //PPS OC4 for A2 
    RPB11Rbits.RPB11R = 0b0101; //PPS OC2 for B11 
    
    OC2CON = 0x0000;            // OC2 Disable
    OC2CON = 0x0006;            // OCM PWM mode   
    OC2RS  = 0;                 // OC2 Secondary Compare Register
                               
    OC4CON = 0x0000;            // OC4 Disable
    OC4CON = 0x0006;            // OCM PWM mode   
    OC4RS  = 0;                 // OC4 Secondary Compare Register
  
    OC2CONbits.ON = 1;          // OC2 Enable
    OC4CONbits.ON = 1;          // OC4 Enable   
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
void ADCInit( void )
{
    AD1CON1 = 0x0000;           // AD1 Enable  

    AD1CON1bits.SSRC = 0b111;   // Auto Convert
    AD1CON1bits.CLRASAM = 1;    // Stop conversions when the first ADC interrupt is generated
                                // Will also clear ASAM bit  
    AD1CON2bits.CSCNA = 1;      // Scan Inputs
    AD1CON2bits.SMPI = 8;       // Irrpt after conversion of 9th sample/convert sequence
    
    AD1CON3bits.SAMC = 26;      // Auto-Sample Time bits - 31 TAD
    AD1CON3bits.ADCS = 4;       // ADC Conversion Clock Select bits - TPB * 2 * (ADCS + 1) = TAD
    
    AD1CHS  = 0;                // Not needed for auto-sampling
                                   
    AD1CSSLbits.CSSL0 = 1;      // Select ANx for input scan - AN0
    AD1CSSLbits.CSSL1 = 1;      // Select ANx for input scan - AN1
    AD1CSSLbits.CSSL6 = 1;      // Select ANx for input scan - AN6
    AD1CSSLbits.CSSL7 = 1;      // Select ANx for input scan - AN7
    AD1CSSLbits.CSSL8 = 1;      // Select ANx for input scan - AN8
    AD1CSSLbits.CSSL9 = 1;      // Select ANx for input scan - AN9 
    AD1CSSLbits.CSSL10 = 1;     // Select ANx for input scan - AN10 
    AD1CSSLbits.CSSL11 = 1;     // Select ANx for input scan - AN11 
    AD1CSSLbits.CSSL12 = 1;     // Select ANx for input scan - AN12
     
    IEC0bits.AD1IE = 0 ;        // AD1 Interrupt Enable Off
    IFS0bits.AD1IF = 0 ;        // AD1 Interrupt Flag Off
    IPC5bits.AD1IP = 3;         // AD1 Interrupt Priority
    IEC0bits.AD1IE = 1;         // AD1 Interrupt Enable On   
    
    AD1CON1bits.ASAM = 1;       // ADC Sample Auto-Start   
    AD1CON1bits.ADON = 1;       // AD1 Enable    
}

//****************************************************************************
// Function:    ICInit
//
// Description: Initializes IC3 and IC4 as to capture posedge of encoders.
//              Initializes IC1 to capture pos and negedges.    
//
// Params:      void
//
// Return:      void
//
//****************************************************************************
void ICInit( void )
{
    // IC4 Motor 2
    IC4R = 7;                   // Input Pin Selection - C5
    IC4CONbits.FEDGE = 1;       // Capture rising edge first
    IC4CONbits.ICTMR = 0;       // Timer 3 Select 
    IC4CONbits.ICM = 0b011;     // Input Capture Mode - Every rising edge
    IC4CONbits.ICI = 0b00;      // Interrupt on every capture event
    
    IEC0bits.IC4IE = 0;         // IC4 Interrupt Enable Off
    IFS0bits.IC4IF = 0;         // IC4 Interrupt Flag Off
    IPC4bits.IC4IP = 4;         // IC4 Interrupt Priority
    
    // IC3 Motor 1
    IC3R = 1;                   // Input Pin Selection - B5 
    IC3CONbits.FEDGE = 1;       // Capture rising edge first
    IC3CONbits.ICTMR = 0;       // Timer 3 Select
    IC3CONbits.ICM = 0b011;     // Input Capture Mode - Every rising edge
    IC3CONbits.ICI = 0b00;      // Interrupt on every capture event
    
    IEC0bits.IC3IE = 0;         // IC3 Interrupt Enable Off
    IFS0bits.IC3IF = 0;         // IC3 Interrupt Flag Off
    IPC3bits.IC3IP = 4;         // IC3 Interrupt Priority

    // IC1
    IC1R = 5;                   // Input Pin Selection - C6 
    IC1CONbits.FEDGE = 1;       // Capture rising edge first
    IC1CONbits.ICTMR = 0;       // Timer 3 Select
    IC1CONbits.ICM = 0b110;     // Input Capture Mode - every edge, specified edge first
    IC1CONbits.ICI = 0b00;      // Interrupt on every capture event
    
    IEC0bits.IC1IE = 0;         // IC1 Interrupt Enable Off
    IFS0bits.IC1IF = 0;         // IC1 Interrupt Flag Off
    IPC1bits.IC1IP = 4;         // IC1 Interrupt Priority
    
    // ENABLE
    IEC0bits.IC1IE = 1;         // IC1 Interrupt Enable
    IEC0bits.IC3IE = 1;         // IC3 Interrupt Enable
    IEC0bits.IC4IE = 1;         // IC4 Interrupt Enable
    
    IC1CONbits.ON = 1;          // IC1 Enable    
    IC3CONbits.ON = 1;          // IC3 Enable
    IC4CONbits.ON = 1;          // IC4 Enable    
}


//****************************************************************************
// Function:    ClearVariables
// 
// Description: Clears flags and other variables
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void ClearVariables( void )
{
    // Clear pins
    LATCbits.LATC7  = 0;    // PCB LED  
    LATAbits.LATA7  = 0;    // Left LED
    LATBbits.LATB9  = 0;    // Right LED
    LATCbits.LATC4  = 0;    // Solenoid Pin Off    
   
    // Clear counts
    FlmDataCnt           = 0;
    USSensEdgeCnt        = 0;
    WaitForAcclCnt       = 0;
    M1PosEdgeCnt         = 0;
    M2PosEdgeCnt         = 0;
    FwdTurnCnt           = 0;
    RvsTurnCnt           = 0;
    WaterPulseCnt        = 0;
    SecCnt               = 0;
    IRSensCnt            = 0;              
    CmpltStopAfterTrnCnt = 0;
    WallFollowInitCnt    = 0;
    MtrsOffCnt           = 0; 
    
    // Clear flags
    SensorEvalFlag      = 0;
    USSensorFlag        = 0;
    AdjustSpeedFlag     = 0;
    ExtinguishFlag      = 0;
    TurnFlag            = 0;    
    FwdTurnDoneFlag     = 0;
    RvsTurnDoneFlag     = 0;
    MapDoneFlag         = 0;
    WaterPulseFlag      = 0;
    M2FasterFlag        = 0;   
    M1FasterFlag        = 0;
    FrntSensFlag        = 0;
    AdjstLeftFlag       = 0;
    AdjstRightFlag      = 0;
    AdjstFwdFlag        = 0;    
    MtrsOffFlag         = 0;
    WFInitFlag          = 0;
    WFRunFlag           = 0;
    CnslChckFlag        = 0;
    WallCllFlag         = 0;  
    DcyCllFlag          = 0;   
    ExitRoomFlag        = 0;    
    IgnFirstFlameFlag   = 0;
    DesignDayBtn        = 0;

    // Clear variables
    Motor1Speed         = 0;
    Motor2Speed         = 0;
    M1Integral          = 0;
    M2Integral          = 0;
    M1Distance          = 0;
    M2Distance          = 0;
    MapIndex            = 0;
    MapDist             = 0;  
    RRIndex             = 0;
    RRDist              = 0;
    FwdTurnDist         = 0;
    RvsTurnDist         = 0;
    USSensPosEdgeTime   = 0;
    USSensNegEdgeTime   = 0;   
    USSensDiff          = 0;
    
    // Reset variables
    FlmMidMin       = SINT16_MAX;
    FlmMidMax       = SINT16_MIN;       
    M1_SuperSlowPWM = SUPER_SLOW_SPEED_INIT;
    M2_SuperSlowPWM = SUPER_SLOW_SPEED_INIT + 40;    
    M1_SlowPWM      = SLOW_SPEED_INIT;
    M2_SlowPWM      = SLOW_SPEED_INIT + 40;
    M1_MedPWM       = MED_SPEED_INIT;
    M2_MedPWM       = MED_SPEED_INIT + 30;      
}

//****************************************************************************
// Function:    MapInit
// 
// Description: Initializes Map Array
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void MapInit( void )
{
    MapDistance[0]  = 40; 
    MapDirection[0] = FORWARD;    
    MapDistance[1]  = 26; 
    MapDirection[1] = FORWARD;     
    MapDistance[2]  = 1;
    MapDirection[2] = FORWARD;      // Check left sensors for Room 1 Door 
    
    MapDistance[3]  = 0;
    MapDirection[3] = LEFT_45;      // Enter room
    
    MapDistance[4]  = 25;
    MapDirection[4] = FORWARD; 
    
    MapDistance[5]  = 0;
    MapDirection[5] = LEFT_45;      // Start center console
    
    MapDistance[6]  = 20;
    MapDirection[6] = FORWARD;
    MapDistance[7]  = 10;
    MapDirection[7] = FORWARD;
    MapDistance[8]  = 5;
    MapDirection[8] = FORWARD;
    
    MapDistance[9]  = 0;
    MapDirection[9] = RIGHT_90;     // First console turn
    
    MapDistance[10]  = 8;
    MapDirection[10] = FORWARD; 
    MapDistance[11]  = 13;
    MapDirection[11] = FORWARD; 
    MapDistance[12]  = 5;
    MapDirection[12] = FORWARD; 
    
    MapDistance[13]  = 0;
    MapDirection[13] = RIGHT_90;    // Second console turn

    MapDistance[14]  = 8;
    MapDirection[14] = FORWARD; 
    MapDistance[15]  = 10;
    MapDirection[15] = FORWARD;     
    MapDistance[16]  = 5;
    MapDirection[16] = FORWARD; 
    
    MapDistance[17]  = 0;     
    MapDirection[17] = RIGHT_90;     // Third console turn 
 
    MapDistance[18]  = 8;
    MapDirection[18] = FORWARD; 
    MapDistance[19]  = 13;
    MapDirection[19] = FORWARD;     
    MapDistance[20]  = 18;
    MapDirection[20] = FORWARD; 
    
    MapDistance[21]  = 0;
    MapDirection[21] = LEFT_90;  
    
    MapDistance[22]  = 28;
    MapDirection[22] = FORWARD;     // Room 1 Exit    
    
    MapDistance[23]  = 0;
    MapDirection[23] = LEFT_90;  
    
    MapDistance[24]  = 10;
    MapDirection[24] = FORWARD;     
    MapDistance[25]  = 53;
    MapDirection[25] = FORWARD;     // Check left sensors for  Room 2 Door    
    MapDistance[26]  = 66;
    MapDirection[26] = FORWARD;     // Check left sensors for Room 3 Door     
    MapDistance[27]  = 50;
    MapDirection[27] = FORWARD;
    
    MapDistance[28]  = 0;
    MapDirection[28] = TURN_180;    // End of Hall
    
    MapDistance[29]  = 50;
    MapDirection[29] = FORWARD;     // Check right sensors for Room 3 Door    
    MapDistance[30]  = 66;
    MapDirection[30] = FORWARD;     // Check right sensors for Room 2 Door 
    MapDistance[31]  = 63;
    MapDirection[31] = FORWARD;     // Check right sensors for Room 1 Door 
    MapDistance[32]  = 13;
    MapDirection[32] = FORWARD;       
    MapDistance[33]  = 0;
    MapDirection[33] = RIGHT_45;    // Small hallway turn
    MapDistance[34]  = 18;
    MapDirection[34] = FORWARD;     
    MapDistance[35]  = 0;
    MapDirection[35] = LEFT_45;     // Small hallway turn    
    MapDistance[36]  = 40;
    MapDirection[36] = FORWARD; 
    MapDistance[37]  = 0;
    MapDirection[37] = FORWARD;     // Back at start
}

//****************************************************************************
// Function:    MotorSpeedCtrl
//
// Description: Sets the Duty Cycle to OC registers
//
// Params:      M1Speed - PWM Duty cycle to control left motor
//              M2Speed - PWM Duty cycle to control right motor
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
// Description: Sets direction of motors
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
            LATBbits.LATB7  = FWD;    
            LATBbits.LATB10 = RVS;                   
            break;
        case FWD:
            LATBbits.LATB7  = RVS;       
            LATBbits.LATB10 = FWD;     
            break;
        case STLL:
            LATBbits.LATB7  = FWD;     
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
//              TrgtEncoder - Target encoder value
//              Motor - Left or right motor selection
//
// Return:      PWM - Adjusted PWM Duty Cycle value
//
//****************************************************************************
uint16 PI( uint16 ActualEncoder, uint16 TrgtEncoder, uint8 Motor )
{
    float Kp = 0.41;  
    float Ki = 0.001;     
    float dt = 0.072;
    
    sint32 error;
    float PWM;
    sint32 Integral;

    error = TrgtEncoder - ActualEncoder;
    Integral = (Motor == MOTOR_1)? M1Integral: M2Integral;
    
    // PI function
    PWM = (Kp * error) + (Ki * Integral);    
  
    // add 0.5 for rounding    
    PWM = (sint16) (( PWM < 0 )? PWM - 0.5: PWM + 0.5);
    
    
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
// Description: Sets the Speed of motors and other variables
//
// Params:      Spd - Speed selection
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
            MotorSpeedCtrl( M1_SuperSlowPWM, M2_SuperSlowPWM );                            
            TargetEncoder = SUPER_SLOW_SPEED;
            MaxPWM = M1_SuperSlowPWM + 150;
            MinPWM = M1_SuperSlowPWM - 150;        
            SpeedCheck = SUPER_SLOW_SPEED_CHK;
            CurrSpeed = SUPER_SLOW;            
            break;            
        case(SLOW):
            MotorSpeedCtrl( M1_SlowPWM, M2_SlowPWM );  
            TargetEncoder = SLOW_SPEED;   
            MaxPWM = M1_SlowPWM + 150;
            MinPWM = M1_SlowPWM - 150;  
            SpeedCheck = SLOW_SPEED_CHK;            
            CurrSpeed = SLOW;
            break;
        case(MED):
            MotorSpeedCtrl( M1_MedPWM, M2_MedPWM );                            
            TargetEncoder = MED_SPEED; 
            MaxPWM = M1_MedPWM + 150;
            MinPWM = M1_MedPWM - 150;    
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

//****************************************************************************
// Function:    SaveCurrEnc
//
// Description: Saves the current PWM duty cycle values of the current speed
//
// Params:      void
//
// Return:      void
//
//****************************************************************************
void SaveCurrEnc( void )
{
    switch (CurrSpeed)
    {
        case(SUPER_SLOW):
            M1_SuperSlowPWM = OC4RS;
            M2_SuperSlowPWM = OC2RS;
            break;
        case(SLOW):
            M1_SlowPWM = OC4RS;
            M2_SlowPWM = OC2RS;            
            break;            
        case(MED):
            M1_MedPWM = OC4RS;
            M2_MedPWM = OC2RS;            
            break;            
    }
}

//****************************************************************************
// Function:    SetDirection
//
// Description: Sets the direction of the motors and of other variables
//
// Params:      Direction - Direction selection
//
// Return:      void
//             
//****************************************************************************
void SetDirection( uint32 Direction )
{
    uint8 TurnVar = 0;
    
    switch(Direction)
    {
        case(FORWARD):
                CurrDir = FORWARD;
                MotorDirectionCtrl( FWD, FWD );                
                TurnFlag = 0;
            break;
        case(REVERSE):
                CurrDir = FORWARD;
                MotorDirectionCtrl( RVS, RVS );   
                TurnFlag = 0;                
            break;  
        case(STALL_M1):      
                MotorDirectionCtrl( STLL, IGN );   
            break;              
        case(STALL_M2):       
                MotorDirectionCtrl( IGN, STLL );   
            break;              
        case(LEFT_20):
                CurrDir = LEFT_20;
                MotorDirectionCtrl( FWD, RVS );  

                FwdTurnCheck = MOTOR_1;                                 
                FwdTurnCnt = TURN_30_ENC;
                RvsTurnCnt = TURN_30_ENC;                
                TurnVar = 1;                
            break;
        case(RIGHT_20):
                CurrDir = RIGHT_20;
                MotorDirectionCtrl( RVS, FWD ); 

                FwdTurnCheck = MOTOR_2;                                                   
                FwdTurnCnt = TURN_30_ENC;
                RvsTurnCnt = TURN_30_ENC;                
                TurnVar = 1;                
            break;
        case(LEFT_45):
                CurrDir = LEFT_45;
                MotorDirectionCtrl( FWD, RVS );  

                FwdTurnCheck = MOTOR_1;                                 
                FwdTurnCnt = TURN_45_ENC;
                RvsTurnCnt = TURN_45_ENC;                
                TurnVar = 1;                
            break;
        case(RIGHT_45):
                CurrDir = RIGHT_45;
                MotorDirectionCtrl( RVS, FWD ); 

                FwdTurnCheck = MOTOR_2;                                                   
                FwdTurnCnt = TURN_45_ENC;
                RvsTurnCnt = TURN_45_ENC;                
                TurnVar = 1;                
            break;
        case(LEFT_50):
                CurrDir = LEFT_50;
                MotorDirectionCtrl( FWD, RVS );  

                FwdTurnCheck = MOTOR_1;                                 
                FwdTurnCnt = TURN_50_ENC;
                RvsTurnCnt = TURN_50_ENC;                
                TurnVar = 1;                
            break;
        case(RIGHT_50):
                CurrDir = RIGHT_50;
                MotorDirectionCtrl( RVS, FWD ); 

                FwdTurnCheck = MOTOR_2;                                                   
                FwdTurnCnt = TURN_50_ENC;
                RvsTurnCnt = TURN_50_ENC;                
                TurnVar = 1;                
            break;            
        case(LEFT_90):
                CurrDir = LEFT_90;
                MotorDirectionCtrl( FWD, RVS );  

                FwdTurnCheck = MOTOR_1;                                 
                FwdTurnCnt = TURN_90_ENC;
                RvsTurnCnt = TURN_90_ENC;                
                TurnVar = 1;                
            break;
        case(RIGHT_90):
                CurrDir = RIGHT_90;
                MotorDirectionCtrl( RVS, FWD ); 

                FwdTurnCheck = MOTOR_2;                                                   
                FwdTurnCnt = TURN_90_ENC;
                RvsTurnCnt = TURN_90_ENC;                
                TurnVar = 1;                
            break;
        case(TURN_180):
                CurrDir = TURN_180;
                MotorDirectionCtrl( RVS, FWD ); 

                FwdTurnCheck = MOTOR_2;                                            
                FwdTurnCnt = TURN_180_ENC;
                RvsTurnCnt = TURN_180_ENC;               
                TurnVar = 1;                
            break;
        case(LEFT_SCAN):
                CurrDir = LEFT_SCAN;
                MotorDirectionCtrl( FWD, RVS );  

                FwdTurnCheck = MOTOR_1;                                 
                FwdTurnCnt = TURN_SCAN_ENC;
                RvsTurnCnt = TURN_SCAN_ENC;
                TurnVar = 1;
            break;
        case(RIGHT_SCAN):
                CurrDir = RIGHT_SCAN;
                MotorDirectionCtrl( RVS, FWD ); 

                FwdTurnCheck = MOTOR_2;                                                   
                FwdTurnCnt = TURN_SCAN_ENC;
                RvsTurnCnt = TURN_SCAN_ENC;                
                TurnVar = 1;                
            break;        
        default:
                MotorDirectionCtrl( FWD, FWD );
                TurnFlag = 0;         
            break;            
    }    
    
    if ( TurnVar != 0 )
    {                 
        M1Distance = 0;
        M2Distance = 0;    
        M1PosEdgeCnt = 0;    
        M2PosEdgeCnt = 0;  
        FwdTurnDoneFlag = 0;
        RvsTurnDoneFlag = 0;        
         
        TurnFlag = 1;                            
    }        
       
}

//****************************************************************************
// Function:    CheckFlameDetectors
//
// Description: Checks if IR Sensors detect a flame higher than 50mV
//
// Params:      void
//
// Return:      FlameDetected - Returns 1 if IR sensors have detected flame
//
//****************************************************************************
uint8 CheckFlameDetectors( void ) 
{    
    uint8 FlameDetected = 0;
    uint32 i; 
    
    for (i = 1; i < 5; i++)
    {       
        if (FlameSens[i] > 15)
        {
            FlameFirstLoc = i;
            FlameDetected = 1;
        }            
    }
  
    return FlameDetected;
} 

//****************************************************************************
// Function:    CenterFlame
//
// Description: Finds which sensor senses the highest flame value
//
// Params:      void
//
// Return:      ActualCenter - Returns the sensor with the highest value
//
//****************************************************************************
uint32 CenterFlame( void )
{
   uint32 ActualCenter = 0;
   uint32 i; 
   uint16 MaxHighest = 0;

    for (i = 0; i < 5; i++)
    {          
        if (MaxHighest < FlameSens[i])
        {
            MaxHighest = FlameSens[i];
            ActualCenter = i;
        }            
    }
   
    // Match up the two sensors on each side of middle sensor
    if (ActualCenter == CENTER_FLAME)
    {
        if ( FlameSens[3] - FlameSens[1] > 2 )
        {
          ActualCenter = 3;  
        }
        if ( FlameSens[1] - FlameSens[3] > 2 )
        {
          ActualCenter = 1;  
        }
    }
   return ActualCenter;
}

//****************************************************************************
// Function:    CheckMap
//
// Description: Checks how much distance has been covered and changes map
//              direction if needed.
//
// Params:      void
//
// Return:      void
//
//****************************************************************************
void CheckMap( void )
{    
    uint8 Skip = 0;
    
    if ( MapDist == MapDistance[MapIndex] && TurnFlag == 0 )
    {
        MapDist = 0;
        M1Distance = 0;
        M2Distance = 0;   

        MapIndex++;    
       
        // Initialize sensor verifications throughout map
        // Wall following, Console edge checking and front sensor distance check
        if ( MapIndex == 7 || MapIndex == 11 || MapIndex == 15 ||  MapIndex == 19 )
        {
            WFInitFlag = 1; // will set WFRun
            WallCheckSide = RIGHT_SIDE;
        }
        else if ( MapIndex == 8 || MapIndex == 12 || MapIndex == 16 ||  MapIndex == 20 )
        {      
            WFRunFlag = 0;
            CnslChckFlag = 1;
        }
        else if ( (MapIndex == 9 || MapIndex == 13 || MapIndex == 17 || MapIndex == 21 ) && (CnslChckFlag != 0) )
        {
            MapIndex--;
            MapDistance[MapIndex]++;
            CnslChckFlag = 1;
            Skip = 1;
        }    
        else if ( MapIndex == 20 ) // check US sensor before turning
        {
            ExitRoomFlag = 1;
        }   
        else if ( MapIndex == 21 && ExitRoomFlag != 0 )
        {       
            MapIndex--;
            MapDistance[MapIndex]++;
            Skip = 1;          
        }
        else if ( (MapIndex == 25) || (MapIndex == 29) )
        {
            WFInitFlag = 1; // will set WFRun   
            WallCheckSide = LEFT_SIDE;            
        }
        else if ( MapIndex == 28 )
        {
            WFRunFlag = 0;            
        }
        
        // Encoder Map
        if (Skip == 0)
        {                      
            if ( MapIndex < MAP_MAX )
            {
                if ( MapDirection[MapIndex] != FORWARD &&  MapDirection[MapIndex] != REVERSE && MapDirection[MapIndex] != DIR_OFF )
                {
                    SaveCurrEnc();               
                    SetDirection( MapDirection[MapIndex] );
                    NextDir = MapDirection[MapIndex+1];  
                    NextSpeed = SLOW;       
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
                MapDoneFlag = 1;
            }
        }
    }
}

//****************************************************************************
// Function:    CheckFrontSensor
//
// Description: Checks front sensor to see if robot is closer than the 
//              distance parameter
//
// Params:      Dist - Distance value to check for
//
// Return:      Cllisn - Returns 1 if robot is closer than distance value.
//
//****************************************************************************
uint8 CheckFrontSensor( uint32 Dist )
{
    uint32 TurnDir; 
    uint8 Cllisn = 0;   
    
    // USSensDiff * 0.0108844 to calculate distance in inches (float)
    if ( USSensDiff <= Dist )
    {
        Cllisn = 1;
        
        if (ExitRoomFlag != 0)
        {
            ExitRoomFlag = 0;
            MapDistance[MapIndex] = MapDist + 1;
        }

        if ( State != FIRE_DETECTED && TurnFlag == 0)
        {

            TurnDir = CheckCollisionSensors();

            SetDirection( TurnDir ); 
            NextDir = FORWARD;
            NextSpeed = CurrSpeed;
        }

    }     
    
    return Cllisn;
}

//****************************************************************************
// Function:    CheckCollisionSensors
//
// Description: Check side sensors to decide which way to turn
//
// Params:      void
//
// Return:      TurnDir - Returns which side to turn to to avoid collision
//
//****************************************************************************
uint32 CheckCollisionSensors( void )
{
    uint32 TurnDir;
    TurnDir = LEFT_90;
    
    if ( (IRSens[0] > 600) || (IRSens[1] > 600) ) // Left Sensors
    {
        TurnDir = RIGHT_90;
    }
    if ( (IRSens[2] > 600) && (IRSens[3] > 600) ) // Right Sensors
    {
        TurnDir = (TurnDir == RIGHT_90)? TURN_180: LEFT_90;
    }   
     
    return TurnDir;
}

//****************************************************************************
// Function:    ReRoute
//
// Description: Reroute for wall collision or decoy avoidence. Returns to 
//              map after completed
//
// Params:      void
//
// Return:      void
//
//****************************************************************************
void ReRoute( void )
{   
    if ( RRDist >= RRouteDistance[RRIndex] && TurnFlag == 0 )
    {
        RRDist = 0;
        M1Distance = 0;
        M2Distance = 0;   
        
        RRIndex++;
        
        if ( RRIndex < REROUTE_MAX )
        {
            if ( RRouteDirection[RRIndex] != FORWARD &&  RRouteDirection[RRIndex] != REVERSE && RRouteDirection[RRIndex] != DIR_OFF )
            {              
                SetDirection( RRouteDirection[RRIndex] );
                          
                NextDir = FORWARD;              
                NextSpeed = SLOW;                 
            }
            else 
            {
                SetSpeed( SLOW );                  
                SetDirection( RRouteDirection[RRIndex] );                                
            }
            
        }
        else
        {            
            SetDirection( STALL_M1 );
            SetDirection( STALL_M2 );
            SetSpeed( OFF );    
            // Clear Reroute Flags
            WallCllFlag = 0; 
            DcyCllFlag = 0;
            // Return map
            MapIndex = SavedMapIndex;
            MapDist = SavedMapDist;
            M1Distance = SavedM1Dist;
            M2Distance = SavedM2Dist;  
            SetDirection(  MapDirection[MapIndex] );           
            SetSpeed( SLOW );  
        }      
    }    
}

//****************************************************************************
// Function:    ShootWater
//
// Description: Sets solenoid pin to high for 100ms
//
// Params:      void
//
// Return:      PulseDne - Returns 1 if solenoid pulse is done
//
//****************************************************************************
uint8 ShootWater( void )
{
    uint8 PulseDne = 0;
    
    // 100 ms pulse    
    if ( WaterPulseFlag == 0 )
    {
        LATCbits.LATC8 = 0;     
        LATCbits.LATC4 = 0;        
        PulseDne = 1;
    }
    else
    {
        LATCbits.LATC8 = 1;       
        LATCbits.LATC4 = 1;
    }
    return PulseDne;
}

//****************************************************************************
// Function:    SetUpWall
//
// Description: Sets up wall collision threshhold to maintain a constant 
//              distance from wall
//
// Params:      Side - Which side to check IR sensors for threshold
//
// Return:      void
//
//****************************************************************************
void SetUpWall( uint8 Side )
{
    uint32 SUCnt = 5;
    uint16 FntSens;
    uint16 BckSens;
    
    FntSens = (Side == LEFT_SIDE)? IRSens[1]: IRSens[3];
    BckSens = (Side == LEFT_SIDE)? IRSens[0]: IRSens[2];
     
    if ((FntSens > 250) &&  (BckSens > 250))
    {       
        WallFollowInitCnt++;
              
        if (WallFollowInitCnt > SUCnt)
        {
            FntThrsh = FntSens;    
            BckThrsh = BckSens;

            WFRunFlag = 1;
            WFInitFlag = 0;        
        }                   
    }
}

//****************************************************************************
// Function:    ConsoleCheck
//
// Description: Checks for edge of console before turning
//
// Params:      void
//
// Return:      void
//
//****************************************************************************
void ConsoleCheck( void )
{
    uint16 FntSens;
    uint16 BckSens;    
    
    FntSens = IRSens[3];
    BckSens = IRSens[2];
   
    if ( ( FntSens <  FntThrsh-100 ) && ( BckSens <  BckThrsh-100 ) )
    {

        MapDistance[MapIndex] = (MapIndex == 20)? MapDist + 16: (MapIndex == 16)? MapDist + 7: MapDist + 4;

        CnslChckFlag = 0;
    }   
}

//****************************************************************************
// Function:    CheckWalls
//
// Description: Check side IR sensors for wall collision based on threshold
//
// Params:      Side - Which side to check IR sensors for threshold
//
// Return:      void        
//
//****************************************************************************
void CheckWalls( uint8 Side )
{
    uint16 FntSens;
    uint16 BckSens;
    
    FntSens = (Side == LEFT_SIDE)? IRSens[1]: IRSens[3];
    BckSens = (Side == LEFT_SIDE)? IRSens[0]: IRSens[2];
         
    if ( ( FntSens >  FntThrsh+20) && ( BckSens >  BckThrsh+20)  && SecCnt == 0 )
    {            
        RRouteDirection[0] = (Side == LEFT_SIDE)? RIGHT_45: LEFT_45;   
        RRouteDistance[1] = 3;
        RRouteDirection[2] = (Side == LEFT_SIDE)? LEFT_45: RIGHT_45;   
        WallCllFlag = 1;
    
        SavedMapIndex = MapIndex;
        SavedMapDist = MapDist;
        SavedM1Dist = M1Distance;
        SavedM2Dist = M2Distance;            
          
        RRDist = 0;
        M1Distance = 0;
        M2Distance = 0;         
        RRIndex = 0;  
        
        SetSpeed( SLOW );                  
        SetDirection( RRouteDirection[RRIndex] );                 
    }    
}

//****************************************************************************
// Function:    DecoyCheck
//
// Description: Stores center flame sensor data for 3 seconds
//
// Params:      void
//
// Return:      ChkCmplt - Returns 1 when 3 seconds of data have been stored.
//
//****************************************************************************
uint8 DecoyCheck( void )
{
    uint8 ChkCmplt = 0;
                    
    if (IgnFirstFlameFlag == 0)
    {
        FlmMidDif = FlameSens[CENTER_FLAME] - PrvMidFlame;
        FlmMidMax  = ( FlmMidDif > FlmMidMax )? FlmMidDif: FlmMidMax;
        FlmMidMin  = ( FlmMidDif < FlmMidMin )? FlmMidDif: FlmMidMin;    
        
        // Should include FireVerifySens() here to shorten decoy check time
    }
    else 
    {
        IgnFirstFlameFlag = 0;
    }
    PrvMidFlame = FlameSens[CENTER_FLAME];                  
                    
    FlmDataCnt++;
    if (FlmDataCnt >= 30381)
    {      
        FlmDataCnt = 0;
        ChkCmplt = 1;
    }      
    return ChkCmplt;
}

//****************************************************************************
// Function:    FireVerifySens
//
// Description: Checks for a flame voltage jump higher than 70 mV 
//
// Params:      void
//
// Return:      FireDetected - Returns 1 if flame is verfied to be a fire
//
//****************************************************************************
uint8 FireVerifySens( void )
{
    uint8 FireDetected = 1;
    sint32 flSensDif;
    flSensDif = FlmMidMax - FlmMidMin;
    
    if (flSensDif > 20)
    {
       FireDetected = 0; 
    }
    
    return FireDetected;    
}

