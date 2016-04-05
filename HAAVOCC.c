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
    LATAbits.LATA0 = 0;
    encCnt = 0;
    while(1)
    { 
        switch(State)
        {
            case START:
                // LED ON

                // start moving
                // Set direction pins and speed
                LATAbits.LATA4 = 0;       // pin 12
                LATBbits.LATB4 = 1;       // pin 11
                LATBbits.LATB7 = 1;       // pin 16
                LATBbits.LATB8 = 0;       // pin 17                              
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
                break;
            case FIRE_EXTINGUISH:
                // fire extinguish function
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
            M1Distance = 0;
            M2Distance = 0;   
            distanceDiff = 0;            
            TurnFlag = 0;
            SpeedUp = 0;
            Debug = 1;
            TEST4 = 0;  
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
       DC = IC4NegEdgeTime - IC4PosEdgeTime; 
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
    
    if (M1Distance > 60000 || M2Distance > 60000)
    {
        M1Distance = (distanceDiff > 0)? distanceDiff: 0;
        M2Distance = (distanceDiff < 0)? -distanceDiff: 0;       
    }
    
    /// fix encoders
    if ( (State == NAVIGATE) && (AdjustSpeedFlag ==  0) && (M1PosEdgeCnt > 0) && (M2PosEdgeCnt > 0) )
    {
        if (CatchUp++ > 3)
        {
//            if (TTFlag != 0 || Fixed == 100)
//            if (SpeedUp != 0)
//            {
//                if ((((M2PosEdgeCnt - M1PosEdgeCnt) < 2) || ((M1PosEdgeCnt - M2PosEdgeCnt) < 2)) )
//                {
//                    TestAdjust++;
//
//                    if (TestAdjust == 200)
//                    {                   
//                        TestAdjust = 0;
//                        LATAbits.LATA0 = 1;                     
//                    }
//                }
//            }
//            if (M1Faster == 1 || M2Faster ==1)  
//            {
//                if ( distDifCnt++ > 100 )
//                {                    
//                    M1Faster = 0;
//                    distanceDiff = M1Distance - M2Distance;                
//                    M2Faster = 0;
//                }
//            }
            if ( (Debug != 0) && (distanceDiff > 15 || distanceDiff < -15))
            {
//                SetSpeed( OFF );
                Debug = 0;
            }           
        
            M1EncCounts[encCnt] = M1PosEdgeCnt;
            M2EncCounts[encCnt] = M2PosEdgeCnt;
        
            M1PWMCounts[encCnt] = Motor1Speed;
            M2PWMCounts[encCnt] = Motor2Speed;            
            
                       
            encCnt++;    
            if (encCnt >= 200)
            {
                encCnt=0;          
            }
            
            if (distanceDiff > 0 && TurnFlag == 0) // Motor 1 faster
            { 
                LATAbits.LATA0 = 1;                 
//
                
                m2target = TargetEncoder+(distanceDiff/3);
                m1target = TargetEncoder-(distanceDiff/3);
            
                Motor2Speed = PI(M2PosEdgeCnt, m2target+1, MOTOR_2);
                Motor1Speed = PI(M1PosEdgeCnt, m1target, MOTOR_1);                  
 
//                if (M1Faster==0)
//                {
//                    turnFixCnt1 = 0;
//                    AfterDiffM1[aft1] = distanceDiff;                  
//                }               
//                else 
//                {
//                    turnFixCnt1++;
//                }
                
                M1Faster = 1;      
                M2Faster = 0;    
//                             
            }
            else if (distanceDiff < 0 && TurnFlag == 0) // Motor 2 faster
            {   
                LATAbits.LATA0 = 1; 
                
                m2target = TargetEncoder+(distanceDiff/3);
                m1target = TargetEncoder-(distanceDiff/3);
            
                Motor2Speed = PI(M2PosEdgeCnt, m2target+1, MOTOR_2);
                Motor1Speed = PI(M1PosEdgeCnt, m1target, MOTOR_1);                                

//                if (M2Faster==0)
//                {
//                    turnFixCnt2 = 0;
//                    AfterDiffM2[aft2] = distanceDiff;                   
//                }  
//                else 
//                {
//                    turnFixCnt2++;
//                }
                
                M2Faster = 1;            
                M1Faster = 0;
            }                    
            else 
            {                           
                LATAbits.LATA0 = 0;   
                
                Motor2Speed = PI(M2PosEdgeCnt, TargetEncoder+1, MOTOR_2);
                Motor1Speed = PI(M1PosEdgeCnt, TargetEncoder, MOTOR_1);    
                
//                if (M1Faster!=0)
//                {
//                    AfterFixM1[aft1] = turnFixCnt1;
//                    aft1 ++;
//                    if (aft1 >=10)
//                    {
//                        aft1 = 0;                        
//                    }                    
//                }
//                if (M2Faster!=0)
//                {
//                    AfterFixM2[aft2] = turnFixCnt2;
//                    aft2 ++;
//                    if (aft2 >=10)
//                    {
//                        aft2 = 0;                        
//                    }                      
//                }    
//                
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
//   LATAbits.LATA0 ^= 1;     
   // 5 Channel sample
   AN1ADC = ADC1BUF0;  
   AN4ADC = ADC1BUF1;  
   AN5ADC = ADC1BUF2;  
   AN9ADC = ADC1BUF3;  
   AN10ADC = ADC1BUF4;  
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
void __ISR (17, IPL4SOFT) IC4IntHandler(void)
{
    IFS0bits.IC4IF = 0;     // Turn Flag Off	 
        
    if (IC4CONbits.ICBNE)
    {
        if (IC4EdgeCnt%2 == 0)
        {
            IC4PosEdgeTime = IC4BUF;       // IC4BUF value is timer value
        }
        else 
        {
            IC4NegEdgeTime = IC4BUF; 
            if ( IC4NegEdgeTime > IC4PosEdgeTime )
            {
                USSensorFlag = 1;
            }
        }
        IC4EdgeCnt++;
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
void __ISR (9, IPL4SOFT) IC2IntHandler(void)
{
    IFS0bits.IC2IF = 0;     // Turn Flag Off	 
        
    if (IC2CONbits.ICBNE)
    {
        PersistantBuffer = IC2BUF;        
        M2PosEdgeCnt++;
    }  
} 
