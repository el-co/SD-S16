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

#pragma config FNOSC = FRCPLL
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLMUL = MUL_20
#pragma config FPLLODIV = DIV_2
#pragma config FPBDIV = DIV_1
#pragma config FSOSCEN = OFF
#pragma config FWDTEN = OFF
#pragma config JTAGEN = OFF

// 40MHz

void main(void)
{
    Init();
    M1Integral = 0;
    M2Integral = 0;
    
    USSensorFlag = 0;
    Fixed = 0;
    SpeedUp = 0;
    maxDistDiff = 0;   
    LATCbits.LATC7 = 0;
    
    LATAbits.LATA7 = 0;     //
    LATBbits.LATB9 = 0;     //
    LATAbits.LATA10 = 0;    //
    LATCbits.LATC8 = 0;     //bottom
    
    //solenoid
    LATCbits.LATC4 = 0;
     
    USInd = 0;
    xin = 0;
    
    flcnt = 0;
    SixtnHz = 0;
    SecCnt = 0;
       
    IRCnt = 0;
    l = 0;   
    
    flmMidMax = 0;
    flmMidMin = 0;
    flcn = 0;
    UnMappedTurn = 0;
    WaterPulseCnt = 0;
    
    RCONbits.BOR = 0;
    RCONbits.POR = 0;
    
    AftTrn = 0;
    tf = 0;
    ss = 0;
    FireCll = 0;
    RRRtrn = 0;
    fwd=0;
    
    WllChck = 0;
//    FireSens = 0;
    
    Sidx = 0;
    filt = 0;
    
    FireLocation = ROOM;
    
    
    WFInit = 0;
    WFRun = 0;    
    DrChck = 0;
    CnslChck = 0;
    
    
    while(1)
    {  
        
        switch(State)
        {
            case IDLE:              
                // Do nothing
                // Push button to go to START
                // select between mode 1 and mode 2                                 
                    if (PORTAbits.RA3 == 0)
                    {
                        Mode = MODE_1;
                        State = START;
                        while(PORTAbits.RA3 == 0);
                        SecCnt = 2;

                    }              

                    if (PORTAbits.RA8 == 0)
                    {
                        Mode = MODE_2;
                        State = START;
//                        Extinguish = 1;
                        while(PORTAbits.RA8 == 0);
                        SecCnt = 2;

                    }                                 

                break;  
            case START:
                // LED ON
                // Set direction pins and speed   
                // Reset values as needed
                CatchUp = 0;
                MapIndex = 0; /// change if fire location
                MapDist = 0;
                MapDone = 0;
                 
                if (SecCnt == 0)
                {                
                    SetSpeed( SLOW ); /// Turn OFF if testing flame sensors
                    SetDirection( FORWARD );

                    RRouteDirection[1] = FORWARD;   
                    RRouteDistance[0] = 0;   
                    RRouteDistance[1] = 3;   
                    RRouteDistance[2] = 0;                      
                    
                    State = NAVIGATE; 
                }
                break;
            case NAVIGATE:
                // 	Use map to navigate

                if (MapDone == 0 && WallCll == 0)
                { 
                    CheckMap();                
                }
                else if (WallCll != 0)
                {
                    ReRoute();
                }
                
                //  Use pi code to adjust speed                  
                if ( AdjustSpeedFlag != 0 )
                {                                   
                    MotorSpeedCtrl( Motor1Speed, Motor2Speed );                   
                    AdjustSpeedFlag = 0;                                   
                }
              
                // front sensor
                if (USSensorFlag != 0)
                {
                    CheckFrontSensor();
                    NextDir = FORWARD;
                    NextSpeed = SLOW;
                    USSensorFlag = 0;
                }
                
                // 	Collision avoidance (ADC)   
                // 	Check IR photo sensors (ADC)                        
                if ( SensorEvalFlag != 0 )
                {              
                    IRCnt++;
                    if (IRCnt%5 == 0)
                    {                             
                        if (WFInit != 0)
                        {                        
                            SetUpWall( WallCheckSide );
                        }
                        if (DrChck != 0)
                        {                        
                            CheckForDoor();
                        }
                        if (CnslChck != 0)
                        {                        
                            ConsoleCheck();
                        }                                                                                 
                        if (WFRun != 0 && WallCll == 0)
                        {
                            CheckWalls( WallCheckSide );   
                        }
   
//                        filt++;
//                               
//                        if (filt % 2 == 0)
//                        {
//                            LF[Sidx] = IRSens[1];
//                            LB[Sidx] = IRSens[0];
//                            RF[Sidx] = IRSens[3];
//                            RB[Sidx] = IRSens[2];                    
//
//                            Sidx++;
//
//                            if ( Sidx >= 3000)
//                            {
//                                Sidx = 0;
//                            }      
//                            filt = 0;
//                        }                    
                        
                        IRCnt = 0;          
                    }  
    
                    if ( CheckFlameDetectors() != 0 )  
                    {
                        SetSpeed( OFF );      
                        ScLeft = 0;
                        ScRight = 0;
                        ScFwd = 0;
                        SetDirection( STALL_M1 );
                        SetDirection( STALL_M2 );                        
                        State = FIRE_DETECTED;
                        
                        //Save for reroute
                        MInd = MapIndex;
                        MDist = MapDist;
                        M1Dist = M1Distance;
                        M2Dist = M2Distance;
                        
                        RRouteDistance[LEFT_REROUTE] = 0;
                        RRouteDistance[RIGHT_REROUTE] = 0;
                        RRouteDistance[FWD_REROUTE] = 0;
                        
//                        State = FIRE_VERIFY;
                    }
//                   ///////////////////////////////////////////////// 
                                   
                    SensorEvalFlag = 0;
                }                

                break;
            case FIRE_DETECTED:
                // Navigate to location         
                // Use IR photo sensors (ADC) to center flame source
                if ( SensorEvalFlag != 0 )
                {                                   
                // Cnter flame and get closer
                    CntrFlame = CenterFlame();
                                                        
                    if ( CntrFlame != CENTER_FLAME )
                    {
                        
                        if ((CntrFlame == 0 || CntrFlame == 1) && ScLeft == 0) // Left Sensors
                        {
                            if (ScRight == 0)
                            {
                                RRouteDistance[FWD_REROUTE] += M1PosEdgeCnt;
                            }
                            ScFwd = 0;
                            ScLeft = 1;
                            ScRight = 0;                            
                            SetDirection( LEFT_SCAN ); 
                            
//                        MFireSens[FireSens] = FlameSens[2];
//                        LFireSens[FireSens] = FlameSens[4];
//                        RFireSens[FireSens] = FlameSens[0];
//                        LMFireSens[FireSens] = FlameSens[3];
//                        RMFireSens[FireSens] = FlameSens[1];
//                        LR[FireSens] = 1;
//                        FireSens++;                            
                            
                        }
                        else if ((CntrFlame == 3 || CntrFlame == 4) && ScRight == 0) // Right Sensors
                        {
                            if (ScLeft == 0)
                            {
                                RRouteDistance[FWD_REROUTE] += M1PosEdgeCnt;
                            }    
                            ScFwd = 0;
                            ScLeft = 0;                            
                            ScRight = 1;
                            SetDirection( RIGHT_SCAN ); 
                            
//                        MFireSens[FireSens] = FlameSens[2];
//                        LFireSens[FireSens] = FlameSens[4];
//                        RFireSens[FireSens] = FlameSens[0];
//                        LMFireSens[FireSens] = FlameSens[3];
//                        RMFireSens[FireSens] = FlameSens[1];
//                        LR[FireSens] = 2;
//                        FireSens++;                            
                            
                        }  
                        SetSpeed( SUPER_SLOW );
                        NextDir = FORWARD;  
                        NextSpeed = OFF; 
                        
                    }
                    else if ( FlameSens[CENTER_FLAME] < 120 ) //smaller if too close and larger if to far
                    {
                        if ( ScLeft != 0 || ScRight != 0 )
                        {
                            switch( CurrDir )
                            {
                                case LEFT_SCAN:
                                {
                                    RRouteDistance[LEFT_REROUTE] += M1Distance;
                                    break;
                                }
                                case RIGHT_SCAN:
                                {
                                    RRouteDistance[RIGHT_REROUTE] += M1Distance;
                                    break;                            
                                }                                              
                            } 
                            
                            RRouteDistance[FWD_REROUTE]-=M1PosEdgeCnt;
                            
                            M1Distance = 0;
                            M2Distance = 0;
                            SetSpeed( SUPER_SLOW );
                            SetDirection( FORWARD );     
                                                    
                            ScLeft = 0;                            
                            ScRight = 0;  
                        }  
                        ScFwd = 1;   
                    }
                    else
                    {
                        if (ScFwd != 0)
                        {
                            RRouteDistance[FWD_REROUTE] += M1PosEdgeCnt;
                        }
                        
                        SetDirection( STALL_M1 );
                        SetDirection( STALL_M2 );
                        SetSpeed(OFF);
                        TurnFlag = 0;
                        ScLeft = 0;
                        ScRight = 0;
                        ScFwd = 0;                        
                        
                         //needed for decoycheck
//                        flmMidMin = 32767;
//                        flmMidMax = -32768;
//                        PrvMidFlame = FlameSens[2];   
                        IgnFirst = 1;                    

                        State = FIRE_VERIFY;                     
                    }

//                    if (FireSens >= 2000 )
//                    {
//                        FireSens = 0;
//                    }                    
                    
                    
                    SensorEvalFlag = 0;
                }
                break;
            case FIRE_VERIFY:
                // use temp sensors to verify flame
                if ( SensorEvalFlag != 0 )
                {                               
                    if ( DecoyCheck() != 0)
                    {
                        if ( FireVerifySens() != 0) // !=
                        {    
                            ss = 0;
                            Extinguish = 1;
                            LATCbits.LATC8 = 1; 
                           
                            State = FIRE_EXTINGUISH;
                        }
                        else 
                        {
                            ss = 0;
                            LATCbits.LATC8 = 0; 
                            flmMidMin = 32767;
                            flmMidMax = -32768;                               
                            
                            RRouteDirection[0] = REVERSE;   
                            RRouteDirection[1] = LEFT_RRT;   
                            RRouteDirection[2] = RIGHT_RRT;                              
                            
                            if (RRouteDistance[LEFT_REROUTE] >= RRouteDistance[RIGHT_REROUTE])
                            {
                                RRouteDistance[LEFT_REROUTE] -= RRouteDistance[RIGHT_REROUTE];
                                RRouteDistance[RIGHT_REROUTE] = 0;
                            }
                            if (RRouteDistance[RIGHT_REROUTE] >= RRouteDistance[LEFT_REROUTE])
                            {
                                RRouteDistance[RIGHT_REROUTE] -= RRouteDistance[LEFT_REROUTE];
                                RRouteDistance[LEFT_REROUTE] = 0;
                            }
                            RRTurn = 0;
                            RRIndex = 0;
                            RRDist = 0;
                            RRCheck = 0;
                            
                            SetSpeed( SLOW );                  
                            SetDirection( RRouteDirection[RRIndex] );                                
                            State = RETURN_ROUTE;  
                            ReturnNav = 1;
                        }                         
                    }
                    
                    SensorEvalFlag = 0;
                }
                
                break;
            case RETURN_ROUTE:
                // return to route
                ReRoute();
                if ( ReturnNav == 0)
                {                    
                    State = NAVIGATE;  
                        
                    RRouteDirection[1] = FORWARD;   
                    RRouteDistance[0] = 0;   
                    RRouteDistance[1] = 3;   
                    RRouteDistance[2] = 0;                          
                        
                    MapIndex = MInd;
                    MapDist = MDist;
                    M1Distance = M1Dist;
                    M2Distance = M2Dist;                                                     
                }
           
                break;
            case FIRE_EXTINGUISH:
                
                // 100ms pulse to solenoid
                if ( Extinguish != 0 )
                {       
                    WaterPulse = 1;
                    // 100ms pulse to solenoid
                    if ( ShootWater() != 0 )
                    {
                        Extinguish = 0;
                        State = IDLE;
                    }
                }       
                
                break;            
            default:
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
// Description: Timer interrupt at 16KHz (62.5us). Used for motor PWM.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void __ISR (8, IPL2SOFT) Timer2IntHandler(void)
{
    IFS0bits.T2IF = 0;      // Turn Flag Off
    
    if (WaterPulse != 0)
    {
        WaterPulseCnt++;
        if (WaterPulseCnt >= 1600)
        {
            WaterPulse = 0;
        }
    }
    
    if (SixtnHz++ >=16000)
    {
        SixtnHz = 0;
        LATAbits.LATA10 ^= 1;
        SecCnt = (SecCnt > 0 )? SecCnt-1: 0;            
    }
    
    if ( State == RETURN_ROUTE && M1Distance >= RRouteDistance[RRIndex] && TurnFlag == 0 )
    {
        RRCheck = 1;
    }    
    
    // Motors
    // 25 1cm
    // 40 1.5cm
    // 400 10cm
    
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

                FwdTurnDone = 1;                
            }
            if ( RvsTurnDist >= RvsTurnCnt )
            {
                Motor2Speed = 0;
                SetDirection( STALL_M2 );
                AdjustSpeedFlag =  1;

                RvsTurnDone = 1;                
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

                FwdTurnDone = 1;
            }
            if ( RvsTurnDist >= RvsTurnCnt )
            {
                Motor1Speed = 0;
                SetDirection( STALL_M1 );
                AdjustSpeedFlag =  1;  

                RvsTurnDone = 1;    
            }            
        }
        
        if ( (FwdTurnDone != 0) && (RvsTurnDone != 0) )
        {      
            if (AftTrn++ > 640)
            {
                AftTrn = 0;
            }

            if ( ( (AftTrn == 0) && ( M1PosEdgeCnt <= 1 ) && ( M2PosEdgeCnt <= 1) ) )
            {
                if ( State == FIRE_DETECTED )      
                {
                    switch( CurrDir )
                    {
                        case LEFT_SCAN:
                        {
                            RRouteDistance[LEFT_REROUTE] += M1Distance;
                            break;
                        }
                        case RIGHT_SCAN:
                        {
                            RRouteDistance[RIGHT_REROUTE] += M1Distance;
                            break;                            
                        }                                              
                    }                                 
                }          
                                                
                SetDirection( NextDir );
                SetSpeed( NextSpeed );
                CatchUp = 0;
                ScLeft = 0;
                ScRight = 0;  
                RRTurn = 1;
                M1Distance = 0;
                M2Distance = 0;   
                
                M1PosEdgeCnt = 0;
                M2PosEdgeCnt = 0;

                distanceDiff = 0;  
                AdjustSpeedFlag = 0;
                M1Integral = 0;
                M2Integral = 0;            
                TurnFlag = 0;
                FwdTurnDone = 0;
                RvsTurnDone = 0; 
                
//                if (CllTurn != 0)
//                {
//                    MapDone = 0;
//                    CllTurn = 0;
//                    MapIndex = MInd;
//                    MapDist = MDist;
//                    M1Distance = M1Dist;
//                    M2Distance = M2Dist;            
//                }                
                
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
void __ISR (12, IPL2SOFT) Timer3IntHandler(void)
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
void __ISR (16, IPL2SOFT) Timer4IntHandler(void)
{
    IFS0bits.T4IF = 0;      // Turn Flag Off
//    if (TEST4++ > 200 && SpeedUp == 0)
//    {
//        
////        SetSpeed(MED); 
////        SetDirection( RIGHT_90 );           
//        SpeedUp = 1;
////        TEST4 = 0;
// 
//    }
   
    if (State == FIRE_DETECTED && ScFwd != 0)
    {
        RRouteDistance[FWD_REROUTE] += M1PosEdgeCnt;

    }  
    
    if (MotorDir == REVERSE && State == NAVIGATE)
    {
        M2Distance -= M2PosEdgeCnt;
        M1Distance -= M1PosEdgeCnt;
    }
    else
    {
        M2Distance += M2PosEdgeCnt;    
        M1Distance += M1PosEdgeCnt;    
    }  
    
    
    distanceDiff = M1Distance - M2Distance;  
    maxDistDiff = ( maxDistDiff < distanceDiff )? distanceDiff: maxDistDiff;    
    maxDistDiffNeg = ( maxDistDiffNeg > distanceDiff )? distanceDiff: maxDistDiffNeg;    

    if ( (M1Distance >= 400) && (M2Distance >= 400) && (TurnFlag == 0) 
            && (State != FIRE_DETECTED) )
    {     
//        while ( (M1Distance >= 400) && (M2Distance >= 400) )
//        {
        
        if (WallCll == 0 && ReturnNav == 0)
        {
            MapDist++;

        }
        else
        {
            RRDist++;            
        }
        
        M1Distance = M1Distance - 400;
        M2Distance = M2Distance - 400;          
//        }
    }
    
//    if ( ( CurrSpeed == SUPER_SLOW ) && ( M1PosEdgeCnt > SUPER_SLOW_SPEED - 5 ) && ( M1PosEdgeCnt < SUPER_SLOW_SPEED + 5 )
//       && ( M2PosEdgeCnt > SUPER_SLOW_SPEED - 5 ) && ( M2PosEdgeCnt < SUPER_SLOW_SPEED + 5 ) )
//    {
//        SetSpeed( SLOW );                    
//    }  

    
    /// fix encoders
    if ( (State == NAVIGATE) && (AdjustSpeedFlag ==  0) 
         && (M1PosEdgeCnt < SpeedCheck) && (M2PosEdgeCnt < SpeedCheck) )
    {
        if ( (M1PosEdgeCnt > 0) && (M2PosEdgeCnt > 0) && (TurnFlag == 0) )
        {
            if ( CatchUp++ > 3 )
            {
                CatchUp--;          
                
                if ( distanceDiff > 0 ) // Motor 1 faster
                { 
                    LATAbits.LATA7 = 1;                 
                    LATBbits.LATB9 = 0; 

                    encAdjust = distanceDiff/3.6;
                    distanceDiff = (sint32) (( encAdjust < 0 )? encAdjust - 0.5: encAdjust + 0.5);
                    
                    m2target = TargetEncoder + distanceDiff;
                    m1target = TargetEncoder - distanceDiff;
//                    
                    Motor2Speed = PI(M2PosEdgeCnt, m2target+2, MOTOR_2);
                    Motor1Speed = PI(M1PosEdgeCnt, m1target, MOTOR_1);                  

                    M1Faster = 1;      
                    M2Faster = 0;                                
                }
                else if ( distanceDiff < 0 ) // Motor 2 faster
                {   
                    LATAbits.LATA7 = 0;                 
                    LATBbits.LATB9 = 1; 

                    encAdjust = distanceDiff/3.6;
                    distanceDiff = (sint32) (( encAdjust < 0 )? encAdjust - 0.5: encAdjust + 0.5);                
                   
                    m2target = TargetEncoder + distanceDiff;
                    m1target = TargetEncoder - distanceDiff;

                    Motor2Speed = PI(M2PosEdgeCnt, m2target+2, MOTOR_2);
                    Motor1Speed = PI(M1PosEdgeCnt, m1target, MOTOR_1);                                

                    M2Faster = 1;            
                    M1Faster = 0;
                }                    
                else 
                {                           
                    LATAbits.LATA7 = 1;   
                    LATBbits.LATB9 = 1; 
                                       
                    Motor2Speed = PI(M2PosEdgeCnt, TargetEncoder+2, MOTOR_2);
                    Motor1Speed = PI(M1PosEdgeCnt, TargetEncoder, MOTOR_1);      

                    M1Faster = 0;
                    M2Faster = 0;                      
                }

                Fixed++;

                AdjustSpeedFlag =  1;
            }
        }
        else if ( TurnFlag == 1 )
        {
            if ( FwdTurnCheck == MOTOR_1 )
            {
                Motor1Speed = (FwdTurnDone != 0)? 0: PI(M1PosEdgeCnt, TargetEncoder, MOTOR_1);
                Motor2Speed = (RvsTurnDone != 0)? 0: PI(M2PosEdgeCnt, TargetEncoder+2, MOTOR_2);                        
            }
            else 
            {
                Motor1Speed = (RvsTurnDone != 0)? 0: PI(M1PosEdgeCnt, TargetEncoder, MOTOR_1);
                Motor2Speed = (FwdTurnDone != 0)? 0: PI(M2PosEdgeCnt, TargetEncoder+2, MOTOR_2); 
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
void __ISR (23, IPL3SOFT) ADCIntHandler(void)
{   
   // 5 Channel sample
    //VERIFY
   FlameSens[0] = ADC1BUF0;  // AN0     RightMost
   FlameSens[1] = ADC1BUF1;  // AN1     RightMid    
   FlameSens[4] = ADC1BUF2;  // AN6     LefttMost
   FlameSens[3] = ADC1BUF3;  // AN7     LefttMid
   IRSens[3] = ADC1BUF4;  // AN9  Right Fwd
   IRSens[0] = ADC1BUF5;  // AN8   Left Back
   IRSens[1] = ADC1BUF6;  // AN11  Left Fwd 
   IRSens[2] = ADC1BUF7;  // AN10  Right Back
   FlameSens[2] = ADC1BUF8;  // AN12    Middle
   
//   ANADC = ADC1BUF5;
//   Buffer value ranges from  0-1023
//   31 per 100 mV
//  
//   1V       310
//   2V       620
//   3V       930
//   3.3V     1023       
   
   SensorEvalFlag = 1;
//   SmplCnt++;
   
   AD1CON1bits.DONE = 0;  
   IFS0bits.AD1IF = 0;      // Turn Flag Off 
   
   AD1CON1bits.ASAM = 1;  
} 

//****************************************************************************
// Function:    IC4IntHandler
// 
// Description: Interrupt for IC4. Stores pos and negedge time of sensor.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void __ISR (5, IPL4SOFT) IC1IntHandler(void)
{
    /*
     49ms refresh rate
     * 147 us = 1 inch
     
     */
    
    
    IFS0bits.IC1IF = 0;     // Turn Flag Off	 
        
    if (IC1CONbits.ICBNE)
    {
        if (IC1EdgeCnt%2 == 0)
        {
            IC1PosEdgeTime = IC1BUF;       // IC4BUF value is timer value
        }
        else 
        {
            IC1NegEdgeTime = IC1BUF; 
            if ( IC1NegEdgeTime > IC1PosEdgeTime )
            {
                USSensDiff = IC1NegEdgeTime - IC1PosEdgeTime;
                USSensorFlag = 1;
            }
        }
        IC1EdgeCnt++;
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
void __ISR (13, IPL4SOFT) IC3IntHandler(void)
{
    IFS0bits.IC3IF = 0;     // Turn Flag Off	 
        
    if (IC3CONbits.ICBNE)
    {
        PersistantBuffer = IC3BUF;
        M1PosEdgeCnt++;
    }  
} 

//****************************************************************************
// Function:    IC2IntHandler
// 
// Description: Interrupt for IC2. Counts posedge of motor encoder.
// 
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void __ISR (17, IPL4SOFT) IC4IntHandler(void)
{
    IFS0bits.IC4IF = 0;     // Turn Flag Off	
    
    if (IC4CONbits.ICBNE)
    {
        PersistantBuffer = IC4BUF;        
        M2PosEdgeCnt++;
    }  
} 

void __ISR (38, IPL3SOFT) I2CMIntHandler(void)
{
    if (IFS1bits.I2C2MIF != 0)
    {
        IFS1bits.I2C2MIF = 0; //sen pen trstat
    }
    if (IFS1bits.I2C2BIF != 0)
    {
        IFS1bits.I2C2BIF = 0;  
    }
     if (IFS1bits.I2C2SIF != 0)
    {
        IFS1bits.I2C2SIF = 0;  
    }
}