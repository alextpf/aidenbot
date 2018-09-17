//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#include "Configuration.h"
#include "HBot.h"
//#include "Camera.h"
//#include "Robot.h"

long curr_time;                 // used in main loop
long prev_time;
int loop_counter;
bool testmode = true;
//
HBot hBot;
//Camera cam;
//Robot robot;

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
  
  Serial.println("AidenBot init:");
  Serial.print("Init pos: x = ");
  Serial.print(ROBOT_INITIAL_POSITION_X);
  Serial.print(", y = ");
  Serial.println(ROBOT_INITIAL_POSITION_Y);
    
  int m1s, m2s;
  HBot::HBotPosToMotorStep(initPos, m1s, m2s);
  
  Serial.print("Motor1 step = ");
  Serial.print( m1s );
  Serial.print(", Motor2 step = ");
  Serial.println( m2s ); Serial.println("");
  
  hBot.GetM1().SetCurrStep( m1s ); // this sets m_CurrStep for Motor1 & Motor2
  hBot.GetM2().SetCurrStep( m2s );
  
  hBot.SetMaxSpeed( MAX_SPEED / 2 );
  hBot.SetMaxAccel( MAX_ACCEL );
  hBot.SetPosStraight( ROBOT_CENTER_X, ROBOT_DEFENSE_POSITION_DEFAULT ); // this sets m_GoalStep for Motor1 & Motor2, and internally set GoalSpeed
  
  prev_time = micros(); 
  hBot.SetTime( prev_time );
  loop_counter=0;
}

void loop() 
{
  curr_time = micros();
  if ( curr_time - prev_time >= 1000 )  // 1Khz loop
  {
    loop_counter++;
//    
//    Serial.print("Counter = ");
//    Serial.println(loop_counter);
//    
    prev_time = curr_time; // update time
    
    if (testmode)
    {
      testMovements();
    }
    
    hBot.Update(); // internally update 
    
  }
}
