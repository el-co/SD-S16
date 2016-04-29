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
    LATCbits.LATC8 = 0;     //
    
    //solenoid
    LATCbits.LATC4 = 0;
     
//    USInd = 0;
    xin = 0;
    
    flcnt = 0;
    SixtnHz = 0;
    SecCnt = 0;
       
    IRCnt = 0;
    RBChng = 0;
    RBState = STEADY;
    RFChng = 0;  
    RFState = STEADY;
    l = 0;   
  
    RBThresh = 0;
    RFThresh = 0;
    ThrshCnt = 0;
    
    flmMidMax = 0;
    flmMidMin = 0;
    flcn = 0;
    UnMappedTurn = 0;
    WaterPulseCnt = 0;
    
    
    RCONbits.BOR = 0;
    RCONbits.POR = 0;
    
    AftTrn = 0;
    tf = 0;
//    WallFollowing = 0;
    cnt = 0; 
    ss = 0;
//    sD = 0;
    while(1)
    { 

//        if (RCONbits.BOR == 1)
//        {   
//            LATCbits.LATC7 = 1;
//
//            bor = 1;
//        }        
        
        switch(State)
        {
            case IDLE:              
                // Do nothing
                // Push button to go to START
                // select between mode 1 and mode 2                                 
//                if (PORTAbits.RA3 == 0)
//                {
//                    if (PORTAbits.RA3 == 0)
//                    {
//                        Mode = MODE_1;
//                        State = START;
//                        SecCnt = 3;
//                        while(PORTAbits.RA3 == 0);
//                    }              
//                }                
//                if (PORTAbits.RA8 == 0)
//                {
//                    if (PORTAbits.RA8 == 0)
//                    {
//                        Mode = MODE_2;
                        State = START;
                        SecCnt = 3;
//                        while(PORTAbits.RA8 == 0);
//                    }              
//                }                     

                break;  
            case START:
                // LED ON
                // Set direction pins and speed   
                // Reset values as needed
                CatchUp = 0;
                MapIndex = 0;
                MapDist = 0;
                MapDone = 0;
                if ( SensorEvalFlag != 0 && ThrshCnt < 21 )
                {              
                    IRCnt++;
                    if (IRCnt%5 == 0)
                    {
                        ThrshCnt++;
                        RBThresh += IRSens[2];
                        RFThresh += IRSens[3];                        
                    }
                }
                 
                if (ThrshCnt >= 20 && SecCnt == 0)
                {                
                    SetSpeed( SLOW );   /// Turn OFF if testing flame sensors
                    SetDirection( FORWARD );                
                

                    RBThresh = RBThresh/20;
                    RFThresh = RFThresh/20; 
//                    WallFollowing = 1;
                    State = NAVIGATE; 
                }
                break;
            case NAVIGATE:
                // 	Use map to navigate
                State = NAVIGATE; 
                // uncomment to follow map
                if (MapDone == 0)
                { 
                    CheckMap();                
                }

                //  Use pi code to adjust speed                  
                if ( AdjustSpeedFlag != 0 )
                {                                   
                    MotorSpeedCtrl( Motor1Speed, Motor2Speed );                   
                    AdjustSpeedFlag = 0;                                   
                }
              
                // front sensor
//                if (USSensorFlag != 0)
//                {
//                    CheckFrontSensor();
//                    NextDir = FORWARD;
//                    NextSpeed = SLOW;
//                    USSensorFlag = 0;
//                }
                
                // 	Collision avoidance (ADC)   
                // 	Check IR photo sensors (ADC)                        
                if ( SensorEvalFlag != 0 )
                {              
                    IRCnt++;
                    if (IRCnt%5 == 0)
                    {   
//                    RB_s[xin] = RBThresh - IRSens[2];                        
//                    RF_s[xin] = RFThresh - IRSens[3];                    
//                    
//                    LB_s[xin] = IRSens[0];                        
//                    LF_s[xin] = IRSens[1];                        
//                    
                    
                    CheckWalls();   
                    
                    if (xin < 6500)
                    {                     
                        RB_s[xin] = M1Wall;                        
                        RF_s[xin] = M2Wall;                          
                        xin++;
                    
 
//                        xin = 0;
//                        l++;
                    }   
                    else 
                    {
                        xin = 0;
                    }
////                    if (l >= 10)
////                    {
////                        l = 0;
////                    }
                    IRCnt = 0;          
                    }
                   
                    // check left sensors for walls 
//                    CheckCollisionSensors();    

                    /////// COMMENT OUT IF FIRE SENSORS NOT CONNECTED
//                    if ( CheckFlameDetectors() != 0 )  
//                    {
//                        SetSpeed( OFF );      
//                        ScLeft = 0;
//                        ScRight = 0;
//                        inc =0;
//                        State = FIRE_DETECTED;
////                        State = FIRE_VERIFY;
//                    }
//                   ///////////////////////////////////////////////// 
                                   
                    SensorEvalFlag = 0;
                }                

                break;
            case FIRE_DETECTED:
                // Navigate to location
                // Keep track of reroute
//                ReRoute();
                // Use IR photo sensors (ADC) to center flame source
                if ( SensorEvalFlag != 0 )
                {                                   
                    // Cnter flame and get closer
                    CntrFlame = CenterFlame();
   
                    if ( CntrFlame != CENTER_FLAME )
                    {
                        if ((CntrFlame == 0 || CntrFlame == 1) && ScLeft == 0) // Left Sensors
                        {
                            ScLeft = 1;
                            ScRight = 0;                            
                            SetDirection( LEFT_SCAN ); 
                            inc = FlameSens[CENTER_FLAME];                            
                        }
                        else if ((CntrFlame == 3 || CntrFlame == 4) && ScRight == 0) // Right Sensors
                        {
                            ScLeft = 0;                            
                            ScRight = 1;
                            SetDirection( RIGHT_SCAN );  
                        }  
                        SetSpeed( SUPER_SLOW );
                        NextDir = FORWARD;  
                        NextSpeed = OFF; 
                        inc = FlameSens[CENTER_FLAME];
                    }
//                    else if ( FlameSens[CENTER_FLAME] < 20 )
//                    {
//                        if (ScRight != 0)
//                        {
//                            SetDirection( RIGHT_SCAN ); 
//                        }
//                        if (ScLeft != 0)
//                        {
//                            SetDirection( LEFT_SCAN ); 
//                        }
//                        
//                        SetSpeed( SUPER_SLOW );
//                        NextDir = FORWARD;  
//                        NextSpeed = OFF;   
//                    }
                    else if ( FlameSens[CENTER_FLAME] < 120 ) //smaller if too close and larger if to far
                    {
//                        sens[ss] = FlameSens[CntrFlame];
//                        sens1[ss] = FlameSens[1];
//                        sens2[ss] = FlameSens[0];
//                        
//                        csens[ss] = CENTER_FLAME;
                        ss++;
                        if (ss>800)
                        {
                           ss = 0;
                        }    
                        
                        if (incCnt++ < 5)
                        {
                            SetSpeed( SUPER_SLOW );
                            SetDirection( FORWARD );
                        }
                        else if ((inc - FlameSens[CntrFlame]) > 3 )
                        {
                            SetDirection( STALL_M1 );
                            SetDirection( STALL_M2 );                            
                            if (ScRight != 0)
                            {
                                SetDirection( RIGHT_SCAN ); 
                            }
                            if (ScLeft != 0)
                            {
                                SetDirection( LEFT_SCAN ); 
                            }

                            SetSpeed( SUPER_SLOW );
                            NextDir = FORWARD;  
                            NextSpeed = OFF;                              
                            
                            incCnt = 0;
                        }
                        else
                        {
                            inc = FlameSens[CntrFlame];
                            incCnt = 0;
                        }
                    }
                    else
                    {
                        SetDirection( STALL_M1 );
                        SetDirection( STALL_M2 );
                        SetSpeed(OFF);
                        TurnFlag = 0;
                        ScLeft = 0;
                        ScRight = 0;
                        I2C2Init(145); 
                        
                        
                         //needed for decoycheck
//                        flmMidMin = 32767;
//                        flmMidMax = -32768;
//                        PrvMidFlame = FlameSens[2];   
//                        IgnFirst = 1;                    

//                        State = NAVIGATE;                        
                        
                        State = FIRE_VERIFY;
                   
                    }


                    SensorEvalFlag = 0;
                }
                break;
            case FIRE_VERIFY:
                // use temp sensors to verify flame
//                if (FireVerifyTemp() != 0)
//                {   
//                    ss = 0;
//                    Extinguish = 1;
//                    State = NAVIGATE; 
////                    State = FIRE_EXTINGUISH;   
//                }
//                else 
//                {
//                    ss = 0;
//                    Extinguish = 0; 
//                    State = NAVIGATE;                      
//                }
                
                if ( SensorEvalFlag != 0 )
                {
                               
                    if ( DecoyCheck() != 0)
                    {
                        if ( FireVerifySens() != 0)
                        {    
                            ss = 0;
                            State = NAVIGATE; 
//                            State = FIRE_EXTINGUISH;
                        }
                        else 
                        {
                            ss = 0;
                            State = NAVIGATE;
//                            State = RETURN_ROUTE;                      
                        }                         
                    }
                    
                    SensorEvalFlag = 0;
                }
                
                break;
            case RETURN_ROUTE:
                // return to route
                if ( ReRoute() == 0 )
                {
                    State = NAVIGATE;
                }
                
                break;
            case FIRE_EXTINGUISH:
                
                // 100ms pulse to solenoid
                // check fire temp to verify fire extinguished
                if (Extinguish != 0)
                {                
                    // 100ms pulse to solenoid
                    if ( ShootWater() != 0 )
                    {
                        if ( FireVerifyTemp() == 0 )
                        {
                            Extinguish = 0;
                        }
                        else
                        {
                            WaterPulse = 1;
                        }
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
    
    if (cnt > 0)
    {
        cnt++;
    }
    
    if (SixtnHz++ >=16000)
    {
        SixtnHz = 0;
        LATAbits.LATA10 ^= 1;
        SecCnt = (SecCnt > 0 )? SecCnt-1: 0;
            
    }
    
    
    // Motors
    // 25 1cm
    // 40 1.5cm
    // 400 10cm
    
    
    if  ( ( State == NAVIGATE || State == FIRE_DETECTED ) && TurnFlag == 1) 
    {
        if (FwdTurnCheck == MOTOR_1)
        {
            FwdTurnDist = M1Distance + M1PosEdgeCnt;
            RvsTurnDist = M2Distance + M2PosEdgeCnt;
            if ( FwdTurnDist >= FwdTurnCnt )
            {
//                if (FwdTurnDone == 0)
//                {
//                    tempFwd = FwdTurnDist;
//                }
//                M1PosEdgeCnt = 0;
                Motor1Speed = 0;
                SetDirection( STALL_M1 );
                AdjustSpeedFlag =  1;  

                FwdTurnDone = 1;                
            }
            if ( RvsTurnDist >= RvsTurnCnt )
            {
//                if (RvsTurnDone == 0)
//                {
//                    tempRvs = RvsTurnDist;
//                }
//                M2PosEdgeCnt = 0;
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
//                M2PosEdgeCnt = 0;
                Motor2Speed = 0;
                SetDirection( STALL_M2 );
                AdjustSpeedFlag =  1;

                FwdTurnDone = 1;
            }
            if ( RvsTurnDist >= RvsTurnCnt )
            {
//                M1PosEdgeCnt = 0;
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
//            if (AftTrn++ > 32)
//            {
//            AftTrn++;
            if ( (AftTrn == 0) && ( M1PosEdgeCnt <= 1 ) && ( M2PosEdgeCnt <= 1) )
            {
                SetDirection( NextDir );
                SetSpeed( NextSpeed );            
                CatchUp = 0;
                ScLeft = 0;
                ScRight = 0;             
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
//                AftTrn = 0;
//                tf = 1;
//            }
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
//        Fixed = 0;
//    }  
    
//    if (M2PosEdgeCnt == 80 && M1PosEdgeCnt == 80)
//    {
//        Fixed = 0;
//    }
    
//    if (tf != 0 )
//    {
//        if (CatchUp == 0)
//        {
//            m1 = M1PosEdgeCnt;
//            m2 = M2PosEdgeCnt;
//        } 
//        else if (CatchUp == 1)
//        {
//            m1_2 = M1PosEdgeCnt;
//            m2_2 = M2PosEdgeCnt;            
//        }
//        else if (CatchUp == 2)
//        {
//            tf =0;      
//        }
//    }    
    
    if (MotorDir == REVERSE)
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

    if ( (M1Distance >= 400) && (M2Distance >= 400) && (TurnFlag == 0))
    {     
//        while ( (M1Distance >= 400) && (M2Distance >= 400) )
//        {
            MapDist++;
            M1Distance = M1Distance - 400;
            M2Distance = M2Distance - 400;    
//        }
    }
//    if ( ( CurrSpeed == SUPER_SLOW ) && ( M1PosEdgeCnt > SUPER_SLOW_SPEED - 5 ) && ( M1PosEdgeCnt < SUPER_SLOW_SPEED + 5 )
//       && ( M2PosEdgeCnt > SUPER_SLOW_SPEED - 5 ) && ( M2PosEdgeCnt < SUPER_SLOW_SPEED + 5 ) )
//    {
//        SetSpeed( SLOW );                    
//    }  
    LATCbits.LATC8 = 0;     //
    
//    if (WallFollowing != 0)
//    {
//        
//        M1Wall = M1Wall;
//        M2Wall = M2Wall;    
//        if ((M1Wall != 0 || M2Wall!=0) && CatchUp >3)
//        {
//        LATCbits.LATC8 = 1;     //
//            
//        }
//            
//    }
//    else
//    {
//        M1Wall = 0;
//        M2Wall = 0;         
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
                    
//                    if (WallFollowing != 0)
//                    {
//                        m2target = TargetEncoder + M2Wall;
//                        m1target = TargetEncoder + M1Wall;                      
//                    }
//                    else
//                    {
                        m2target = TargetEncoder + distanceDiff;
                        m1target = TargetEncoder - distanceDiff;
//                    }
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

//                    if (WallFollowing != 0)
//                    {
//                        m2target = TargetEncoder + M2Wall;
//                        m1target = TargetEncoder + M1Wall;                      
//                    }
//                    else
//                    {                    
                        m2target = TargetEncoder + distanceDiff;
                        m1target = TargetEncoder - distanceDiff;
//                    }
                    Motor2Speed = PI(M2PosEdgeCnt, m2target+2, MOTOR_2);
                    Motor1Speed = PI(M1PosEdgeCnt, m1target, MOTOR_1);                                

                    M2Faster = 1;            
                    M1Faster = 0;
                }                    
                else 
                {                           
                    LATAbits.LATA7 = 1;   
                    LATBbits.LATB9 = 1; 
                    
//                    if (WallFollowing != 0)
//                    {
//                        m2target = TargetEncoder + M2Wall;
//                        m1target = TargetEncoder + M1Wall;                      
//                    }
//                    else
//                    {                    
//                        m2target = TargetEncoder+2;
//                        m1target = TargetEncoder;
//                    }                    
                    Motor2Speed = PI(M2PosEdgeCnt, TargetEncoder+2, MOTOR_2);
                    Motor1Speed = PI(M1PosEdgeCnt, TargetEncoder, MOTOR_1);      

                    M1Faster = 0;
                    M2Faster = 0;                      
                }

                Fixed++;

                if ( xinp < 45 )
                {
                        RBPI_s[xinp] = M1Wall;                        
                        RFPI_s[xinp] = M2Wall;                          
                        xinp++;           
                }

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