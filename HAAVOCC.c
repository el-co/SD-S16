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
    
    Hz = 0.02;
    TM364PS = 625000;
    Input = TM364PS * Hz;
    USSensorFlag = 0;
    Fixed = 0;
    SensorCnt = 0;
    Debug = 0;
    SpeedUp = 0;
    TestAdjust = 0;    
    distDifCnt = 0;
    maxDistDiff = 0;  
    aft1 = 0;
    aft2 = 0;  
    turnFixCnt1 = 0;
    turnFixCnt2 = 0;    
    LATCbits.LATC7 = 0;
    encCnt = 0;
    while(1)
    { 
        switch(State)
        {
            case START:
                // LED ON
                // Set direction pins and speed                           
                CatchUp = 0;
                SetSpeed( SLOW );  
                SetDirection( FORWARD );                
                State = NAVIGATE; 
                break;
            case NAVIGATE:
                if ( AdjustSpeedFlag != 0 )
                {
                    MotorSpeedCtrl( Motor1Speed, Motor2Speed );
                    
                    AdjustSpeedFlag = 0;
                }
                if ( SensorEvalFlag != 0 )
                {
//                    SensorCalc();                 
                    SensorEvalFlag = 0;
                }                
                
                State = NAVIGATE; 
                // navigate function
                break;
            case FIRE_DETECTED:
                // fire detected function
                break;
            case FIRE_VERIFY:
                // fire verify function
                if (FireVerify(VRFY_FIRE) != 0)
                {
                    Extinguish = 1;
                    State = FIRE_EXTINGUISH;   
                }
                else 
                {
                    State = NAVIGATE;                      
                }
                
                break;
            case FIRE_EXTINGUISH:
                // fire extinguish function
                // 100ms pulse to solenoid
                while (Extinguish != 0)
                {
                    
                    // 100ms pulse to solenoid
                    
                    if ( FireVerify( VRFY_FIRE_EXT ) != 0 )
                    {
                        Extinguish = 0;
                    }
                }       
                
                break;
            case IDLE:
                // idle function
                break;              
            default:
                State = START;
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

    if  (State == NAVIGATE && TurnFlag == 1) 
    {
        MotorTurnDistance = (MotorTurnCheck == MOTOR_1)? (M1Distance + M1PosEdgeCnt): (M2Distance + M2PosEdgeCnt);
        MotorTurnDistance -= StartTurnCnt;

        if ( MotorTurnDistance >= TurnCnt )
        {
            SetDirection( FORWARD );
            SetSpeed( OFF );
            M1Distance = 0;
            M2Distance = 0;   
            distanceDiff = 0;            
            TurnFlag = 0;
//            SpeedUp = 0;
            Debug = 1;
//            TEST4 = 0;  
        }
    }
} 

//****************************************************************************
// Function:    Timer3IntHandler
// 
// Description: Timer interrupt at 21Hz (47.6ms).  Used for IC4 of sensor. 
//                      ~should be 49ms
// Params:      void
// 
// Return:      void
// 
//****************************************************************************
void __ISR (12, IPL2SOFT) Timer3IntHandler(void)
{
    IFS0bits.T3IF = 0;      // Turn Flag Off

    if (USSensorFlag != 0)
    {
       DC = IC1NegEdgeTime - IC1PosEdgeTime; 
       CalcSensPer = DC/Input;
    }
  
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
//    if (TEST4++ > 25 && SpeedUp == 0)
//    {
//        
////        SetSpeed(MED); 
//        SetDirection( TURN_180 );           
//        SpeedUp = 1;
////        TEST4 = 0;
// 
//        Fixed = 0;
//    }  
    
    M2Distance += M2PosEdgeCnt;    
    M1Distance += M1PosEdgeCnt;    
    
    distanceDiff = M1Distance - M2Distance;  
    maxDistDiff = ( maxDistDiff < distanceDiff )? distanceDiff: maxDistDiff;    
    maxDistDiffNeg = ( maxDistDiffNeg > distanceDiff )? distanceDiff: maxDistDiffNeg;    

    if (M1Distance > 60000 || M2Distance > 60000)
    {
        M1Distance = (distanceDiff > 0)? distanceDiff: 0;
        M2Distance = (distanceDiff < 0)? -distanceDiff: 0;       
    }
    
    /// fix encoders
    if ( (State == NAVIGATE) && (AdjustSpeedFlag ==  0) 
         && (M1PosEdgeCnt > 0) && (M1PosEdgeCnt < SpeedCheck)  
         && (M2PosEdgeCnt > 0) && (M2PosEdgeCnt < SpeedCheck) )
    {
        if (CatchUp++ > 3)
        {
            CatchUp--;
            
            MdistDiff[encCnt] = distanceDiff;
            M1EncCounts[encCnt] = M1PosEdgeCnt;
            M2EncCounts[encCnt] = M2PosEdgeCnt;
        
            M1PWMCounts[encCnt] = Motor1Speed;
            M2PWMCounts[encCnt] = Motor2Speed;            
                                   
            encCnt++;    
            if (encCnt >= 1002)
            {
                encCnt=0;          
            }
            
            if (distanceDiff > 0 && TurnFlag == 0) // Motor 1 faster
            { 
                LATCbits.LATC7 = 1;                 
                
                encAdjust = distanceDiff/3;
                distanceDiff = (sint32) (( encAdjust < 0 )? encAdjust - 0.5: encAdjust + 0.5);
                
                m2target = TargetEncoder + distanceDiff;
                m1target = TargetEncoder - distanceDiff;
            
                Motor2Speed = PI(M2PosEdgeCnt, m2target, MOTOR_2);
                Motor1Speed = PI(M1PosEdgeCnt, m1target, MOTOR_1);                  
                
                M1Faster = 1;      
                M2Faster = 0;                                
            }
            else if (distanceDiff < 0 && TurnFlag == 0) // Motor 2 faster
            {   
                LATCbits.LATC7 = 1; 

                encAdjust = distanceDiff/3;
                distanceDiff = (sint32) (( encAdjust < 0 )? encAdjust - 0.5: encAdjust + 0.5);                
                
                m2target = TargetEncoder + distanceDiff;
                m1target = TargetEncoder - distanceDiff;
            
                Motor2Speed = PI(M2PosEdgeCnt, m2target, MOTOR_2);
                Motor1Speed = PI(M1PosEdgeCnt, m1target, MOTOR_1);                                
               
                M2Faster = 1;            
                M1Faster = 0;
            }                    
            else 
            {                           
                LATCbits.LATC7 = 0;   
                
                Motor2Speed = PI(M2PosEdgeCnt, TargetEncoder, MOTOR_2);
                Motor1Speed = PI(M1PosEdgeCnt, TargetEncoder, MOTOR_1);      
                
                M1Faster = 0;
                M2Faster = 0;                      
            }
            
            lastM1 = M1PosEdgeCnt;
            lastM2 = M2PosEdgeCnt;
              
            Fixed++;
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
//   LATCbits.LATC7 ^= 1;     
   // 5 Channel sample
   AN0ADC = ADC1BUF0;  
   AN1ADC = ADC1BUF1;  
   AN4ADC = ADC1BUF2;  
   AN5ADC = ADC1BUF3;  
   AN6ADC = ADC1BUF4;  
   AN9ADC = ADC1BUF5;  
   AN10ADC = ADC1BUF6;  
   AN11ADC = ADC1BUF7;  
   AN12ADC = ADC1BUF8;     
//   ANADC = ADC1BUF5;
//   Buffer value ranges from  0-1023
//   31 per 100 mV
//  
//   1V       310
//   2V       620
//   3V       930
//   3.3V     1023       
   
   SensorEvalFlag = 1;
   
   
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
//    LATCbits.LATC7 ^= 1;      
        
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
    
//    LATCbits.LATC7 ^= 1;      
    if (IC4CONbits.ICBNE)
    {
        PersistantBuffer = IC4BUF;        
        M2PosEdgeCnt++;
    }  
} 
