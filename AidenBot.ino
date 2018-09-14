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

void setup() 
{
  Serial.begin(BAUD_RATE);
  delay(5000);
  
  SetPINS();
  
  // Enable Motors
  digitalWrite(X_ENABLE_PIN    , LOW);
  digitalWrite(Y_ENABLE_PIN    , LOW);

  SetTimerInterrupt();

  // init params
  
  HBot hBot;
  
  hBot.SetPosInternal( ROBOT_INITIAL_POSITION_X, ROBOT_INITIAL_POSITION_Y );
  hBot.SetMaxSpeed( MAX_SPEED / 2 );
  hBot.SetMaxAccel( MAX_ACCEL );
  hBot.SetPosStraight( ROBOT_CENTER_X, ROBOT_DEFENSE_POSITION_DEFAULT );
  
  Camera cam;
  Robot robot;

}

void loop() 
{
  
}
