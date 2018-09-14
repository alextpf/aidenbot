//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#include "Configuration.h"
#include "HBot.h"
#include "Camera.h"
#include "Robot.h"

long curr_time;                 // used in main loop
long prev_time;
long loop_counter;
bool testmode = true;

HBot hBot;
Camera cam;
Robot robot;

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
    
  int m1s, m2s;
  HBot::HBotPosToMotorStep(initPos, m1s, m2s);
  
  hBot.GetM1().SetStep( m1s ); // this sets m_Step for Motor1 & Motor2
  hBot.GetM2().SetStep( m2s );
  
  hBot.SetMaxSpeed( MAX_SPEED / 2 );
  hBot.SetMaxAccel( MAX_ACCEL );
  hBot.SetPosStraight( ROBOT_CENTER_X, ROBOT_DEFENSE_POSITION_DEFAULT ); // this sets GoalStep for Motor1 & Motor2, and internally set GoalSpeed
  
  prev_time = micros(); 
  loop_counter=0;
}

void loop() 
{
  curr_time = micros();
  if ( curr_time - prev_time >= 1000 )  // 1Khz loop
  {
    loop_counter++;
    
    prev_time = curr_time; // update time
    
    if (testmode)
    {
      testMovements();
    }
    
    hBot.Update();
    
//    led_test();
  }
}
