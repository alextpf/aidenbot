//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#include "Configuration.h"
#include "HBot.h"
#include "PacketReader.h"
//#include "Camera.h"
//#include "Robot.h"

long curr_time;                 // used in main loop
long prev_time;
bool testmode = true;
//
HBot hBot;
PacketReader reader;
//Camera cam;
//Robot robot;

//#define DEBUG_PACKET_READER

void setup() 
{
  Serial.begin(BAUD_RATE);
  delay(5000);
  
  SetPINS();
  
  // Enable Motors
  digitalWrite(X_ENABLE_PIN    , LOW);
  digitalWrite(Y_ENABLE_PIN    , LOW);
  
  SetTimerInterrupt();

  // init HBot params
  RobotPos initPos( ROBOT_INITIAL_POSITION_X, ROBOT_INITIAL_POSITION_Y ); // mm
  hBot.SetRobotPos( initPos );
  
  #ifdef SHOW_LOG
      // log
      Serial.println("AidenBot init:");
      Serial.print("Init pos: x = ");
      Serial.print(ROBOT_INITIAL_POSITION_X);
      Serial.print(", y = ");
      Serial.println(ROBOT_INITIAL_POSITION_Y);
      //=============================================
  #endif
  
  long m1s, m2s;
  HBot::HBotPosToMotorStep(initPos, m1s, m2s);
  
  #ifdef SHOW_LOG
      // log
      Serial.print("Motor1 step = ");
      Serial.print( m1s );
      Serial.print(", Motor2 step = ");
      Serial.println( m2s ); Serial.println("");
      //=============================================
  #endif
  
  hBot.GetM1().SetCurrStep( m1s ); // this sets m_CurrStep for Motor1 & Motor2
  hBot.GetM2().SetCurrStep( m2s );
  
  hBot.SetMaxAbsSpeed( MAX_ABS_SPEED );
  hBot.SetMaxAbsAccel( MAX_ABS_ACCEL );
  hBot.SetPosStraight( ROBOT_CENTER_X, ROBOT_DEFENSE_POSITION_DEFAULT ); // this sets m_GoalStep, and internally set m_AbsGoalSpeed for M1 & M2
//  hBot.SetPosStraight(ROBOT_MAX_X, ROBOT_MAX_Y); // upper right
  
  prev_time = micros(); 
  hBot.SetTime( prev_time );
}

void loop() 
{
  curr_time = micros();
  if ( curr_time - prev_time >= 1000  /*&& hBot.GetLoopCounter()< 20*/ )  // 1Khz loop
  {
    prev_time = curr_time; // update time
    
    if (testmode)
    {
      testMovements();
    }
    
    if ( hBot.GetLoopCounter() % 10 == 0 )
    {
      hBot.UpdatePosStraight();  // update straight line motion algorithm
    }
    
    hBot.Update(); // internally update 
    // reader.ReadPacket();
#ifdef DEBUG_PACKET_READER
    Serial.println( "Desired bot pos = " );
    Serial.println( reader.GetDesiredBotPos().m_X );
    Serial.println( reader.GetDesiredBotPos().m_Y );

    Serial.println( "Detected bot pos = " );
    Serial.println( reader.GetDetectedBotPos().m_X );
    Serial.println( reader.GetDetectedBotPos().m_Y );

    Serial.println( "Desired Motor Speed = " );
    Serial.println( reader.GetDesiredMotorSpeed() );
#endif
  } 
}
