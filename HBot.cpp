//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////

#include "HBot.h"
#include "Configuration.h"
#include "Util.h"

//=========================================================
HBot::HBot()
: m_Time( 0 )
{}

//=========================================================
HBot::~HBot()
{}

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

//=========================================================
void HBot::Update() // aka positionControl()
{
  m_Pos = MotorStepToHBotPos( m_M1.GetStep(), m_M2.GetStep() ); // update m_Pos

  // record time
  uint32_t currTime = micros();
  int dt = currTime - m_Time;
  dt = constrain( currTime - m_Time, 0, 2000 );
  
  m_Time = currTime; // update time

  // update motor acceleration
  m_M1.UpdateAccel();
  m_M2.UpdateAccel();
  
  m_M1.UpdateSpeed( dt );  
  m_M2.UpdateSpeed( dt );

  int8_t dir = m_M1.GetDir();
  int s = m_M1.GetSpeed();
  
  if ( s > 0 && dir == 1 )
  {
    SET(PORTF,1);
  }
  else if ( s < 0 && dir == -1 )
  {
    CLR(PORTF,1);
  }

  OCR1A = m_M1.GetPeriod();

  dir = m_M2.GetDir();
  s = m_M2.GetSpeed();
  
  if ( s > 0 && dir == 1 )
  {
    SET(PORTF,7);
  }
  else if ( s < 0 && dir == -1 )
  {
    CLR(PORTF,7);
  }

  OCR3A = m_M2.GetPeriod();

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
  
  m_M1.SetGoalStep( m1s );
  m_M2.SetGoalStep( m2s );
  
} // SetPosInternal

//=========================================================
void HBot::UpdatePosStraight()
{  
  // Speed adjust to draw straight lines (aproximation)
  // First, we calculate the distante to target on each axis
  int diff_M1 = myAbs( m_M1.GetGoalStep() - m_M1.GetStep() );
  int diff_M2 = myAbs( m_M2.GetGoalStep() - m_M2.GetStep() );
  
  // Now, we calculate the factor to apply to draw straight lines. Speed adjust based on target distance
  float factor1 = 1.0;
  float factor2 = 1.0;
  if (diff_M2 == 0) // to avoid division by 0
  {
    factor2 = 0.0;
  }
  else if (diff_M1 > diff_M2)
  {
    factor2 = (float)diff_M2 / (float)diff_M1;
  }
  else
  {
    factor1 = (float)diff_M1 / (float)diff_M2;
  }
  
  // Calculate the target speed (with sign) for each motor
  long tspeed1 = sign( m_M1.GetGoalStep() - m_M1.GetStep() ) * GetMaxSpeed() * factor1; // arduino "long" is 32 bit
  long tspeed2 = sign( m_M2.GetGoalStep() - m_M2.GetStep() ) * GetMaxSpeed() * factor2; // arduino "long" is 32 bit
  
  // Now we calculate a compensation factor. This factor depends on the acceleration of each motor (difference on speed we need to apply to each motor)
  // This factor was empirically tested (with a simulator) to reduce overshoots
  long diffspeed1 = abs( m_M1.GetSpeed() - tspeed1);
  long diffspeed2 = abs( m_M2.GetSpeed() - tspeed2);
  
  float speedfactor1 = 1.05 - (diffspeed2 - diffspeed1) / (2.0 * GetMaxSpeed());  
  speedfactor1 = constrain( speedfactor1, 0.0, 1.0 );
  
  float speedfactor2 = 1.05 - (diffspeed1 - diffspeed2) / (2.0 * GetMaxSpeed());
  speedfactor2 = constrain( speedfactor2, 0.0, 1.0 );

  // Set motor speeds. We apply the straight factor and the "acceleration compensation" speedfactor
  const int target_speed_M1 = GetMaxSpeed() * factor1 * speedfactor1 * speedfactor1;
  const int target_speed_M2 = GetMaxSpeed() * factor2 * speedfactor2 * speedfactor2;
  
  m_M1.SetGoalSpeed( target_speed_M1 );
  m_M2.SetGoalSpeed( target_speed_M2 );  
} // UpdatePosStraight

//=========================================================
void HBot::SetPosStraight( int x, int y )
{  
  SetPosInternal( x, y );
  UpdatePosStraight();  
} // SetPosStraight
