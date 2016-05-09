//****************************************************************************
//Filename:		HAAVOCC.c
//
//Description:	HAAVOCC Main Software
//
//Date:         03.12.16
//
//Complier:		XC32
//
//****************************************************************************

#include "HAAVOCC.h"

#pragma config FNOSC    = FRCPLL
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLMUL  = MUL_20
#pragma config FPLLODIV = DIV_2
#pragma config FPBDIV   = DIV_1
#pragma config FSOSCEN  = OFF
#pragma config FWDTEN   = OFF
#pragma config JTAGEN   = OFF

// 40MHz Clock

void main( void )
{
    Init();
 
    LATCbits.LATC7  = 0;    // PCB LED  
    LATAbits.LATA7  = 0;    // Left LED
    LATBbits.LATB9  = 0;    // Right LED
    LATAbits.LATA10 = 0;    // Top LED
    LATCbits.LATC8  = 0;    // Bottom LED
    LATCbits.LATC4  = 0;    // Solenoid Pin Off
   
    RCONbits.BOR    = 0;    // Clear Brown Out Reset
    RCONbits.POR    = 0;    // Clear Power On Reset
 
    SixtnHzCnt = 0;         // Start Heartbeat
    ClearVariables();       // Clear Variables
  
    while(1)
    {  
        switch(State)
        {
            case IDLE:              
                // Push Buttons
                LATCbits.LATC8 = 1;
                if (PORTAbits.RA3 == 0) // BATTLE BUTTON
                {
                    DesignDayBtn    = 1;
                    ExtinguishFlag  = 1;           
                    State = FIRE_EXTINGUISH;
                    while(PORTAbits.RA3 == 0);
                }              
                if (PORTAbits.RA8 == 0) // START BUTTON
                {
                    DesignDayBtn = 0;
                    State = START;
                    while(PORTAbits.RA8 == 0);
                    SecCnt = 2;
                }                                 
                break;  
            case START:
                LATCbits.LATC8 = 0;
                
                // Reset values as needed                
                ClearVariables();               
                MapInit();
                
                // Set direction pins and speed                   
                SetSpeed( OFF );    
                SetDirection( FORWARD );      
            
                if (SecCnt == 0)
                {                
                    // Reset Reroute values
                    RRouteDirection[1]  = FORWARD;   
                    RRouteDistance[0]   = 0;   
                    RRouteDistance[1]   = 3;   
                    RRouteDistance[2]   = 0;                      

                    // Start Map
                    SetSpeed( SLOW );
                    SetDirection( MapDirection[MapIndex] );
                    
                    State = NAVIGATE; 
                }
                break;
            case NAVIGATE:
                // 	Use map to navigate
                if (MapDoneFlag == 0 && WallCllFlag == 0)
                { 
                    CheckMap();                
                }
                else if (WallCllFlag != 0)
                {
                    ReRoute();
                }
                
                //  Use PI to adjust speed                  
                if ( AdjustSpeedFlag != 0 )
                {                                   
                    MotorSpeedCtrl( Motor1Speed, Motor2Speed );                   
                    AdjustSpeedFlag = 0;                                   
                }
              
                // Check if motors still running
                if ( (M1PosEdgeCnt == 0) &&  (M2PosEdgeCnt == 0))
                {
                    MtrsOffFlag = 1;
                }
                else 
                {
                    MtrsOffCnt  = 0;
                    MtrsOffFlag = 0;
                }
                
                // UltraSonic front sensor
                if (USSensorFlag != 0)
                {
                    if (ExitRoomFlag != 0)
                    {
                        CheckFrontSensor(TWO_FT);                        
                    }
                    if (FrntSensFlag != 0)
                    {
                       CheckFrontSensor(ONE_FT);  
                    }
                    USSensorFlag = 0;
                }
                
                // 	Check IR photo and IR distance sensors (ADC)                        
                if ( SensorEvalFlag != 0 )
                {              
                    IRSensCnt++;
                    if (IRSensCnt%5 == 0)
                    {                             
                        if (WFInitFlag != 0)
                        {                        
                            SetUpWall( WallCheckSide );
                        }
                        if (CnslChckFlag != 0)
                        {                        
                            ConsoleCheck();
                        }                                                                                 
                        if (WFRunFlag != 0 && WallCllFlag == 0)
                        {
                            CheckWalls( WallCheckSide );   
                        }
                                           
                        IRSensCnt = 0;          
                    }  
    
                    if ( CheckFlameDetectors() != 0 && SecCnt == 0 )  
                    {
                        SetSpeed( OFF );  
                        SetDirection( STALL_M1 );
                        SetDirection( STALL_M2 );                       
                                         
                        //Save for Reroute
                        SavedMapIndex   = MapIndex;
                        SavedMapDist    = MapDist;
                        SavedM1Dist     = M1Distance;
                        SavedM2Dist     = M2Distance;

                        AdjstLeftFlag   = 0;
                        AdjstRightFlag  = 0;
                        AdjstFwdFlag    = 0;                        
                        State   = FIRE_DETECTED;                        
                    }
                                   
                    SensorEvalFlag = 0;
                }                

                break;
            case FIRE_DETECTED:

                // Use IR photo sensors (ADC) to center flame source
                if ( SensorEvalFlag != 0 )
                {                                   
                    // Cnter flame and get closer
                    CntrFlame = CenterFlame();

                    if ( CntrFlame != CENTER_FLAME )
                    {
                        if ((CntrFlame == 0 || CntrFlame == 1) && AdjstLeftFlag == 0) // Left Sensors
                        {
                            AdjstFwdFlag    = 0;
                            AdjstLeftFlag   = 1;
                            AdjstRightFlag  = 0;                            
                            SetDirection( LEFT_SCAN ); 

                        }
                        else if ((CntrFlame == 3 || CntrFlame == 4) && AdjstRightFlag == 0) // Right Sensors
                        {

                            AdjstFwdFlag    = 0;
                            AdjstLeftFlag   = 0;                            
                            AdjstRightFlag  = 1;
                            SetDirection( RIGHT_SCAN );                           

                        }  
                        SetSpeed( SUPER_SLOW );
                        NextDir = FORWARD;  
                        NextSpeed = OFF; 

                    }
                    else if ( FlameSens[CENTER_FLAME] < 120 )
                    {
                        if ( AdjstLeftFlag != 0 || AdjstRightFlag != 0 || AdjstFwdFlag == 0 )
                        {                      
                            M1Distance = 0;
                            M2Distance = 0;
                            SetSpeed( SUPER_SLOW );
                            SetDirection( FORWARD );     

                            AdjstLeftFlag   = 0;                            
                            AdjstRightFlag  = 0;  
                        }  
                        AdjstFwdFlag = 1;   
                    }
                    else
                    {
                        SetDirection( STALL_M1 );
                        SetDirection( STALL_M2 );
                        SetSpeed(OFF);
                        
                        TurnFlag = 0;
                        AdjstLeftFlag   = 0;
                        AdjstRightFlag  = 0;
                        AdjstFwdFlag    = 0;                        
                        IgnFirstFlameFlag = 1;                    

                        State = FIRE_VERIFY;                     
                    }

                    SensorEvalFlag = 0;
                }
                break;
            case FIRE_VERIFY:
                // Use IR photo sensors to verify flame
                if ( SensorEvalFlag != 0 )
                {                               
                    if ( DecoyCheck() != 0)
                    {
                        if ( FireVerifySens() != 0) // !=
                        {    
                            ExtinguishFlag = 1;
                            LATCbits.LATC8 = 1; 
                           
                            State = FIRE_EXTINGUISH;
                        }
                        else 
                        {
                            LATCbits.LATC8 = 0; 
                            FlmMidMin = SINT16_MAX;
                            FlmMidMax = SINT16_MIN;                               
                          
                            // Set up Reroute map
                            RRouteDirection[1]  = FORWARD;   
                            RRouteDistance[0]   = 0;   
                            RRouteDistance[2]   = 0;   

                            FireSide = (FlameFirstLoc == 2 || FlameFirstLoc ==  3 || FlameFirstLoc ==  4 )? RIGHT_SIDE: LEFT_SIDE;
                            RRouteDirection[0]  = (FireSide == LEFT_SIDE)? RIGHT_45: LEFT_45;  
                            RRouteDistance[1]   = 8;
                            RRouteDirection[2]  = (FireSide == LEFT_SIDE)? LEFT_45: RIGHT_45;   
                            
                            DcyCllFlag  = 1;        
                            M1Distance  = 0;
                            M2Distance  = 0;         
                            RRIndex     = 0;  
                            RRDist      = 0;

                            // Start Reroute
                            SetSpeed( SLOW );                  
                            SetDirection( RRouteDirection[RRIndex] ); 

                            State = RETURN_ROUTE;  
                        }                         
                    }
                    
                    SensorEvalFlag = 0;
                }
                
                break;
            case RETURN_ROUTE:
                // Return to route
                if (DcyCllFlag != 0)
                {
                    ReRoute();
                }
                else 
                {
                    // Return to Navigat and ignore sensors for 7 seconds
                    SecCnt = 7;
                    FrntSensFlag = 0;
                    State = NAVIGATE;                       
                }
           
                break;
            case FIRE_EXTINGUISH:
                
                // 100ms pulse to solenoid
                if ( ExtinguishFlag != 0 )
                {       
                    WaterPulseFlag = 1;
                    if ( ShootWater() != 0 )
                    {
                        // Ignore CheckFlameDetectors() on Design Day
                        if ( (CheckFlameDetectors() == 0) || (DesignDayBtn != 0) )
                        {                         
                            ExtinguishFlag = 0;
                            State   = IDLE;
                        }
                        else
                        {                           
                            SecCnt  = 15;
                            State   = FIRE_DETECTED;
                        }
                    }
                }       
                
                break;            
            default:
                SetSpeed( OFF );    
                SetDirection( FORWARD );                     
                State = IDLE;
                break;
        }                    
    }
}

//****************************************************************************
//
//                              Interrupt Functions  
// 
//      __ISR ( Vector( Data Sheet Pg 64), IPLXSOFT(X is Priority Number) )
//****************************************************************************

//****************************************************************************
// Function:    Timer2IntHandler
// 
// Description: Timer interrupt at 16KHz (62.5us).
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void __ISR (8, IPL2SOFT) Timer2IntHandler( void )
{
    IFS0bits.T2IF = 0;      // Turn Flag Off
    
    if (WaterPulseFlag != 0)
    {
        WaterPulseCnt++;
        if (WaterPulseCnt >= 1600) // 100 ms water pulse
        {
            WaterPulseCnt   = 0;
            WaterPulseFlag  = 0;
        }
    }
    
    if ( MtrsOffFlag != 0 )
    {
        MtrsOffCnt++;
        if (MtrsOffCnt > 320000) // 20 Sec of no motor mvmnt in Navigate State
        {
            MtrsOffFlag = 0;
            MtrsOffCnt  = 0;
            State = IDLE;
        }
    }
    
    if (SixtnHzCnt++ >= 16000) // 1 Sec Heartbeat
    {
        SixtnHzCnt = 0;
        LATAbits.LATA10 ^= 1;
        SecCnt = (SecCnt > 0 )? SecCnt-1: 0; // Second Countdown     
    }
    
    if  ( ( State == NAVIGATE || State == FIRE_DETECTED || State == RETURN_ROUTE) && TurnFlag == 1) 
    {
        if (FwdTurnCheck == MOTOR_1)
        {
            FwdTurnDist = M1Distance + M1PosEdgeCnt;
            RvsTurnDist = M2Distance + M2PosEdgeCnt;
            if ( FwdTurnDist >= FwdTurnCnt )
            {
                Motor1Speed = 0;
                SetDirection( STALL_M1 );
                AdjustSpeedFlag =  1;  

                FwdTurnDoneFlag = 1;                
            }
            if ( RvsTurnDist >= RvsTurnCnt )
            {
                Motor2Speed = 0;
                SetDirection( STALL_M2 );
                AdjustSpeedFlag =  1;

                RvsTurnDoneFlag = 1;                
            }
            
        }                          
        else
        {
            FwdTurnDist = M2Distance + M2PosEdgeCnt;              
            RvsTurnDist = M1Distance + M1PosEdgeCnt;
            if ( FwdTurnDist >= FwdTurnCnt )
            {
                Motor2Speed = 0;
                SetDirection( STALL_M2 );
                AdjustSpeedFlag =  1;

                FwdTurnDoneFlag = 1;
            }
            if ( RvsTurnDist >= RvsTurnCnt )
            {
                Motor1Speed = 0;
                SetDirection( STALL_M1 );
                AdjustSpeedFlag =  1;  

                RvsTurnDoneFlag = 1;    
            }            
        }
        
        if ( (FwdTurnDoneFlag != 0) && (RvsTurnDoneFlag != 0) )
        {      
            // Wait 40ms to make sure motors have stopped turning
            if (CmpltStopAfterTrnCnt++ > 640)
            {
                CmpltStopAfterTrnCnt = 0;
            }

            if ( (CmpltStopAfterTrnCnt == 0) && ( M1PosEdgeCnt <= 1 ) && ( M2PosEdgeCnt <= 1) )
            {      
                // Clear Flags
                AdjstLeftFlag   = 0;
                AdjstRightFlag  = 0;  
                AdjustSpeedFlag = 0;
                FwdTurnDoneFlag = 0;
                RvsTurnDoneFlag = 0;                 
                TurnFlag = 0;
                
                // Clear Motor and PI values
                M1Distance  = 0;
                M2Distance  = 0;   
                M1Integral  = 0;
                M2Integral  = 0;                    
                M1PosEdgeCnt = 0;
                M2PosEdgeCnt = 0;
                DistanceDiff = 0;  
                WaitForAcclCnt = 0;
    
                // Start map again
                SetDirection( NextDir );
                SetSpeed( NextSpeed );                
            }
        }
    }
} 

//****************************************************************************
// Function:    Timer3IntHandler
// 
// Description: Timer interrupt at 21Hz (49 ms).  Used for IC4 of sensor. 
//                      ~should be 49ms
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void __ISR (12, IPL2SOFT) Timer3IntHandler( void )
{
    IFS0bits.T3IF = 0;      // Turn Flag Off
  
}

//****************************************************************************
// Function:    Timer4IntHandler
// 
// Description: Timer interrupt at 10Hz (100ms).  Used for IC of encoders. 
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void __ISR (16, IPL2SOFT) Timer4IntHandler( void )
{
    IFS0bits.T4IF = 0;      // Turn Flag Off

    if (CurrDir == REVERSE && State == NAVIGATE)
    {
        M2Distance -= M2PosEdgeCnt;
        M1Distance -= M1PosEdgeCnt;
    }
    else
    {
        M2Distance += M2PosEdgeCnt;    
        M1Distance += M1PosEdgeCnt;    
    }  
      
    // 400 Encoder ticks add 1 to MapDist/RRDist variables
    if ( (M1Distance >= 400) && (M2Distance >= 400) && (TurnFlag == 0) 
            && (State != FIRE_DETECTED) )
    {     
        if (WallCllFlag == 0 && DcyCllFlag == 0)
        {
            MapDist++;
        }
        else
        {
            RRDist++;            
        }
        
        M1Distance = M1Distance - 400;
        M2Distance = M2Distance - 400;          
    }

    DistanceDiff = M1Distance - M2Distance;  
    
    /// Fix encoders
    if ( (State == NAVIGATE) && (AdjustSpeedFlag ==  0) 
         && (M1PosEdgeCnt < SpeedCheck) && (M2PosEdgeCnt < SpeedCheck) )
    {
        // Use PI if not turning and motors on
        if ( (M1PosEdgeCnt > 0) && (M2PosEdgeCnt > 0) && (TurnFlag == 0) )
        {
            if ( WaitForAcclCnt++ > 3 )
            {
                WaitForAcclCnt--;          
                
                if ( DistanceDiff > 0 ) // Motor 1 faster
                { 
                    LATAbits.LATA7 = 1;                 
                    LATBbits.LATB9 = 0; 

                    // Adjust target encoder value to make up for distance diff
                    EncAdjust = DistanceDiff/3.6;
                    DistanceDiff = (sint32) (( EncAdjust < 0 )? EncAdjust - 0.5: EncAdjust + 0.5);                  
                    M2AdjustedTargetEnc = TargetEncoder + DistanceDiff;
                    M1AdjustedTargetEnc = TargetEncoder - DistanceDiff;
                    
                    // Calculate PI
                    Motor2Speed = PI(M2PosEdgeCnt, M2AdjustedTargetEnc+2, MOTOR_2);
                    Motor1Speed = PI(M1PosEdgeCnt, M1AdjustedTargetEnc, MOTOR_1);                  

                    M1FasterFlag = 1;      
                    M2FasterFlag = 0;                                
                }
                else if ( DistanceDiff < 0 ) // Motor 2 faster
                {   
                    LATAbits.LATA7 = 0;                 
                    LATBbits.LATB9 = 1; 

                    // Adjust target encoder value to make up for distance diff                    
                    EncAdjust = DistanceDiff/3.6;
                    DistanceDiff = (sint32) (( EncAdjust < 0 )? EncAdjust - 0.5: EncAdjust + 0.5);                                
                    M2AdjustedTargetEnc = TargetEncoder + DistanceDiff;
                    M1AdjustedTargetEnc = TargetEncoder - DistanceDiff;
          
                    // Calculate PI
                    Motor2Speed = PI(M2PosEdgeCnt, M2AdjustedTargetEnc+2, MOTOR_2);
                    Motor1Speed = PI(M1PosEdgeCnt, M1AdjustedTargetEnc, MOTOR_1);                                

                    M1FasterFlag = 0;                    
                    M2FasterFlag = 1;            
                }                    
                else // Motors Equal
                {                           
                    LATAbits.LATA7 = 1;   
                    LATBbits.LATB9 = 1; 
                                       
                    // Calculate PI
                    Motor2Speed = PI(M2PosEdgeCnt, TargetEncoder+2, MOTOR_2);
                    Motor1Speed = PI(M1PosEdgeCnt, TargetEncoder, MOTOR_1);      

                    M1FasterFlag = 0;
                    M2FasterFlag = 0;                      
                }

                AdjustSpeedFlag =  1;
            }
        }
        else if ( TurnFlag == 1 )
        {
            // Only use PI if wheel is not done turning 
            if ( FwdTurnCheck == MOTOR_1 )
            {
                Motor1Speed = (FwdTurnDoneFlag != 0)? 0: PI(M1PosEdgeCnt, TargetEncoder, MOTOR_1);
                Motor2Speed = (RvsTurnDoneFlag != 0)? 0: PI(M2PosEdgeCnt, TargetEncoder+2, MOTOR_2);                        
            }
            else 
            {
                Motor1Speed = (RvsTurnDoneFlag != 0)? 0: PI(M1PosEdgeCnt, TargetEncoder, MOTOR_1);
                Motor2Speed = (FwdTurnDoneFlag != 0)? 0: PI(M2PosEdgeCnt, TargetEncoder+2, MOTOR_2); 
            } 
            AdjustSpeedFlag =  1;            
        }            
    }   
    
    M2PosEdgeCnt = 0;
    M1PosEdgeCnt = 0;   
}

//****************************************************************************
// Function:    ADC2IntHandler
// 
// Description: ADC Interrupt reads 5 channels.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void __ISR (23, IPL3SOFT) ADCIntHandler( void )
{   
   FlameSens[0] = ADC1BUF0; // AN0  Right Sensor
   FlameSens[1] = ADC1BUF1; // AN1  Right Mid Sensor
   FlameSens[4] = ADC1BUF2; // AN6  Left Sensor
   FlameSens[3] = ADC1BUF3; // AN7  Left Mid Sensor
   IRSens[3]    = ADC1BUF4; // AN9  Right Fwd Sensor
   IRSens[0]    = ADC1BUF5; // AN8  Left Back Sensor
   IRSens[1]    = ADC1BUF6; // AN11 Left Fwd Sensor
   IRSens[2]    = ADC1BUF7; // AN10 Right Back Sensor
   FlameSens[2] = ADC1BUF8; // AN12 Middle Sensor
     
   AD1CON1bits.DONE = 0;  
   IFS0bits.AD1IF = 0;      // Turn Flag Off  
   
   AD1CON1bits.ASAM = 1;  
   SensorEvalFlag = 1;
} 

//****************************************************************************
// Function:    IC1IntHandler
// 
// Description: Interrupt for IC1. Stores pos and negedge time of sensor.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void __ISR (5, IPL4SOFT) IC1IntHandler( void )
{
    IFS0bits.IC1IF = 0;     // Turn Flag Off	 
        
    if (IC1CONbits.ICBNE)
    {
        if (USSensEdgeCnt%2 == 0)
        {
            USSensPosEdgeTime = IC1BUF;  // Pos Edge is specified first
        }
        else 
        {
            USSensNegEdgeTime = IC1BUF; 
            // Check that Neg edge happened second            
            if ( USSensNegEdgeTime > USSensPosEdgeTime ) 
            {
                USSensDiff = USSensNegEdgeTime - USSensPosEdgeTime;
                USSensorFlag = 1;
            }
        }
        
        USSensEdgeCnt++;
    }  
} 

//****************************************************************************
// Function:    IC3IntHandler
// 
// Description: Interrupt for IC3. Counts posedge of motor encoder.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void __ISR (13, IPL4SOFT) IC3IntHandler( void )
{
    IFS0bits.IC3IF = 0;     // Turn Flag Off	 
        
    if (IC3CONbits.ICBNE)
    {
        PersistantBuffer = IC3BUF;
        M1PosEdgeCnt++;
    }  
} 

//****************************************************************************
// Function:    IC4IntHandler
// 
// Description: Interrupt for IC4. Counts posedge of motor encoder.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void __ISR (17, IPL4SOFT) IC4IntHandler( void )
{
    IFS0bits.IC4IF = 0;     // Turn Flag Off	
    
    if (IC4CONbits.ICBNE)
    {
        PersistantBuffer = IC4BUF;        
        M2PosEdgeCnt++;
    }  
} 