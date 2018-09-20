//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////

#include "HBot.h"
#include "Configuration.h"
#include "Util.h"

////////////////////////////////////////////
// Utility functions
////////////////////////////////////////////
//=========================================================
RobotPos HBot::MotorStepToHBotPos(int m1Step, int m2Step)
{
  RobotPos pos;
  pos.m_X = ( m1Step + m2Step ) * 0.5 / X_AXIS_STEPS_PER_UNIT;
  pos.m_Y = ( m1Step - m2Step ) * 0.5 / Y_AXIS_STEPS_PER_UNIT;
  return pos;
} // MotorStepToHBotPos

//=========================================================
void HBot::HBotPosToMotorStep(const RobotPos& pos, int& m1Step, int& m2Step)
{
  m1Step = (pos.m_X + pos.m_Y) * X_AXIS_STEPS_PER_UNIT;
  m2Step = (pos.m_X - pos.m_Y) * Y_AXIS_STEPS_PER_UNIT;
} // HBotPosToMotorStep

////////////////////////////////////////////
// class functions
////////////////////////////////////////////
//=========================================================
HBot::HBot()
: m_Time( 0 )
{}

//=========================================================
HBot::~HBot()
{}

//=========================================================
void HBot::Update() // aka positionControl()
{  
  // convert from motor steps to robot position
  m_Pos = MotorStepToHBotPos( m_M1.GetCurrStep(), m_M2.GetCurrStep() ); // update m_Pos

      //log...
    //  Serial.print("Current BotPos: x = ");
    //  Serial.print(m_Pos.m_X);
    //  Serial.print(", y = ");
    //  Serial.println(m_Pos.m_Y);
      //===========================
      
  // record time
  uint32_t currTime = micros();

      //log...
    //  Serial.print("currTime - m_Time =  ");
    //  Serial.println(currTime - m_Time);
      //===========================
  uint16_t dt = constrain( currTime - m_Time, 0, 2000 ); // Limit dt (it should be around 1000 most times, 1ms)
  
  m_Time = currTime; // update time

  // update motor acceleration
  m_M1.UpdateAccel(); // update m_Accel for M1 & M2
  m_M2.UpdateAccel();
  
  m_M1.UpdateSpeed( dt, Motor::MOTOR_NUM::M1 ); // update m_CurrSpeed, m_Dir, m_Period for M1 & M2
  m_M2.UpdateSpeed( dt, Motor::MOTOR_NUM::M2 );

} // Update

//=========================================================
void HBot::SetPosInternal( int x, int y )
{    
  // Constrain to robot limits...  
  RobotPos goal(
    constrain( x, ROBOT_MIN_X, ROBOT_MAX_X ), 
    constrain( y, ROBOT_MIN_Y, ROBOT_MAX_Y ) ); // mm

  int m1s, m2s;
  HBotPosToMotorStep(goal, m1s, m2s);

  #ifdef SHOW_LOG
      // log
      Serial.print("SetPosInternal: Motor1 step = ");
      Serial.print( m1s );
      Serial.print(", Motor2 step = ");
      Serial.println( m2s );
      //====================================
  #endif
    
  m_M1.SetGoalStep( m1s ); // set m_GoalStep
  m_M2.SetGoalStep( m2s );
  
} // SetPosInternal

//=========================================================
void HBot::UpdatePosStraight()
{  
  // Speed adjust to draw straight lines (aproximation)
  // First, we calculate the distante to target on each axis
  const int diff_M1 = m_M1.GetGoalStep() - m_M1.GetCurrStep();
  const int diff_M2 = m_M2.GetGoalStep() - m_M2.GetCurrStep();

  const unsigned int absDiffM1 = abs( diff_M1 );
  const unsigned int absDiffM2 = abs( diff_M2 );

  #ifdef SHOW_LOG
      // log
      Serial.println("UpdatePosStraight: ");
      Serial.print("m1 goal step = ");
      Serial.print( m_M1.GetGoalStep() );
      Serial.print(", m1 curr step = ");
      Serial.println( m_M1.GetCurrStep() );
      
      Serial.print("m2 goal step = ");
      Serial.print( m_M2.GetGoalStep() );
      Serial.print(", m2 curr step = ");
      Serial.println( m_M2.GetCurrStep() );  Serial.println("");

      Serial.print("diff_M1 = ");
      Serial.print( diff_M1 );
      Serial.print(", diff_M2 = ");
      Serial.println( diff_M2 );

      Serial.print("absDiffM1 = ");
      Serial.print( absDiffM1 );
      Serial.print(", absDiffM2 = ");
      Serial.println( absDiffM2 );
      //===================================
  #endif
  
  // Now, we calculate the factor to apply to draw straight lines. Speed adjust based on target distance
  float factor1 = 1.0;
  float factor2 = 1.0;
  if ( absDiffM2 == 0 ) // to avoid division by 0
  {
    factor2 = 0.0;
  }
  else if ( absDiffM1 > absDiffM2 )
  {
    factor2 = (float)absDiffM2 / (float)absDiffM1;
  }
  else
  {
    factor1 = (float)absDiffM1 / (float)absDiffM2;
  }

  #ifdef SHOW_LOG
      // log
      Serial.print("factor1 = ");
      Serial.print( factor1 );
      Serial.print(", factor2 = ");
      Serial.println( factor2 );Serial.println( "" );
      //=====================================
  #endif
  
  // Calculate the target speed (with sign) for each motor
  long targetSpeed1 = sign( diff_M1 ) * GetMaxAbsSpeed() * factor1; // arduino "long" is 32 bit
  long targetSpeed2 = sign( diff_M2 ) * GetMaxAbsSpeed() * factor2; // arduino "long" is 32 bit
  
  // Now we calculate a compensation factor. This factor depends on the acceleration of each motor (difference on speed we need to apply to each motor)
  // This factor was empirically tested (with a simulator) to reduce overshoots
  const unsigned int diffspeed1 = abs( m_M1.GetCurrSpeed() - targetSpeed1);
  const unsigned int diffspeed2 = abs( m_M2.GetCurrSpeed() - targetSpeed2);

  #ifdef SHOW_LOG
      // log
      Serial.print("M1 curr speed = ");
      Serial.print( m_M1.GetCurrSpeed() );
      Serial.print(", targetSpeed1 = ");
      Serial.println( targetSpeed1 );
      Serial.print("M2 curr speed = ");
      Serial.print( m_M2.GetCurrSpeed() );
      Serial.print(", targetSpeed2 = ");
      Serial.println( targetSpeed2 );  
      Serial.println( "" );
      //=====================================
  #endif
  
  float tmp = ((float)diffspeed2 - (float)diffspeed1) / (2.0 * (float)GetMaxAbsSpeed());
        
  float speedfactor1 = 1.05 - tmp;  
  speedfactor1 = constrain( speedfactor1, 0.0, 1.0 );
  
  float speedfactor2 = 1.05 + tmp;
  speedfactor2 = constrain( speedfactor2, 0.0, 1.0 );

  // Set motor speeds. We apply the straight factor and the "acceleration compensation" speedfactor
  const int target_speed_M1 = sign( diff_M1 ) * GetMaxAbsSpeed() * factor1 * speedfactor1 * speedfactor1;
  const int target_speed_M2 = sign( diff_M2 ) * GetMaxAbsSpeed() * factor2 * speedfactor2 * speedfactor2;

  #ifdef SHOW_LOG
      // log
      Serial.print("M1 target speed = ");
      Serial.print( target_speed_M1 );
      Serial.print(", speedfactor1 = ");
      Serial.println( speedfactor1 );
      Serial.print("M2 target speed = ");
      Serial.print( target_speed_M2 );
      Serial.print(", speedfactor2 = ");
      Serial.println( speedfactor2 );  
      Serial.println( "" );
      //=====================================
  #endif
  
  m_M1.SetGoalSpeed( target_speed_M1 ); // set m_GoalSpeed
  m_M2.SetGoalSpeed( target_speed_M2 );  
} // UpdatePosStraight

//=========================================================
void HBot::SetPosStraight( int x, int y )
{  
  #ifdef SHOW_LOG
      //log  
      Serial.println("SetPosStraight: x = ");
      Serial.print( x );
      Serial.print(", y = ");
      Serial.println( y ); Serial.println("");
      //========================
  #endif
      
  SetPosInternal( x, y ); // set m_GoalStep for M1 & M2
  UpdatePosStraight(); // use algorithm to calculate goal speed and set m_GoalSpeed for M1 & M2
} // SetPosStraight
