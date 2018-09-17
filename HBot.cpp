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
  m_Pos = MotorStepToHBotPos( m_M1.GetCurrStep(), m_M2.GetCurrStep() ); // update m_Pos

  //log...
//  Serial.print("Current BotPos: x = ");
//  Serial.print(m_Pos.m_X);
//  Serial.print(", y = ");
//  Serial.println(m_Pos.m_Y);
  //===========================
  
  // record time
  uint32_t currTime = micros();
  int dt = currTime - m_Time;

  //log...
//  Serial.print("dt =  ");
//  Serial.println(dt);
  //===========================
  dt = constrain( dt, 0, 2000 );
  
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

  // log
//  Serial.print("SetPosInternal: Motor1 step = ");
//  Serial.print( m1s );
//  Serial.print(", Motor2 step = ");
//  Serial.println( m2s );
  //====================================
    
  m_M1.SetGoalStep( m1s ); // set m_GoalStep
  m_M2.SetGoalStep( m2s );
  
} // SetPosInternal

//=========================================================
void HBot::UpdatePosStraight()
{  
  // Speed adjust to draw straight lines (aproximation)
  // First, we calculate the distante to target on each axis
  int diff_M1 = myAbs( m_M1.GetGoalStep() - m_M1.GetCurrStep() );
  int diff_M2 = myAbs( m_M2.GetGoalStep() - m_M2.GetCurrStep() );

  // log
//  Serial.println("UpdatePosStraight: ");
//  Serial.print("m1 goal step = ");
//  Serial.print( m_M1.GetGoalStep() );
//  Serial.print(", m1 curr step = ");
//  Serial.println( m_M1.GetCurrStep() );
//  
//  Serial.print("m2 goal step = ");
//  Serial.print( m_M2.GetGoalStep() );
//  Serial.print(", m2 curr step = ");
//  Serial.println( m_M2.GetCurrStep() );  Serial.println("");
  //===================================
  
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
  
  // log
//  Serial.print("factor1 = ");
//  Serial.print( factor1 );
//  Serial.print(", factor2 = ");
//  Serial.println( factor2 );Serial.println( "" );
  //=====================================
  
  // Calculate the target speed (with sign) for each motor
  long tspeed1 = sign( diff_M1 ) * GetMaxSpeed() * factor1; // arduino "long" is 32 bit
  long tspeed2 = sign( diff_M2 ) * GetMaxSpeed() * factor2; // arduino "long" is 32 bit
  
  // Now we calculate a compensation factor. This factor depends on the acceleration of each motor (difference on speed we need to apply to each motor)
  // This factor was empirically tested (with a simulator) to reduce overshoots
  long diffspeed1 = abs( m_M1.GetCurrSpeed() - tspeed1);
  long diffspeed2 = abs( m_M2.GetCurrSpeed() - tspeed2);
  
  // log
//  Serial.print("M1 curr speed = ");
//  Serial.print( m_M1.GetCurrSpeed() );
//  Serial.print(", tspeed1 = ");
//  Serial.println( tspeed1 );
//  Serial.print("M2 curr speed = ");
//  Serial.print( m_M2.GetCurrSpeed() );
//  Serial.print(", tspeed2 = ");
//  Serial.println( tspeed2 );  
//  Serial.println( "" );
  //=====================================
  
  float speedfactor1 = 1.05 - (diffspeed2 - diffspeed1) / (2.0 * GetMaxSpeed());  
  speedfactor1 = constrain( speedfactor1, 0.0, 1.0 );
  
  float speedfactor2 = 1.05 - (diffspeed1 - diffspeed2) / (2.0 * GetMaxSpeed());
  speedfactor2 = constrain( speedfactor2, 0.0, 1.0 );

  // Set motor speeds. We apply the straight factor and the "acceleration compensation" speedfactor
  const int target_speed_M1 = GetMaxSpeed() * factor1 * speedfactor1 * speedfactor1;
  const int target_speed_M2 = GetMaxSpeed() * factor2 * speedfactor2 * speedfactor2;
  
  // log
//  Serial.print("M1 target speed = ");
//  Serial.print( target_speed_M1 );
//  Serial.print(", speedfactor1 = ");
//  Serial.println( speedfactor1 );
//  Serial.print("M2 target speed = ");
//  Serial.print( target_speed_M2 );
//  Serial.print(", speedfactor2 = ");
//  Serial.println( speedfactor2 );  
//  Serial.println( "" );
  //=====================================
  
  m_M1.SetGoalSpeed( target_speed_M1 ); // set m_GoalSpeed
  m_M2.SetGoalSpeed( target_speed_M2 );  
} // UpdatePosStraight

//=========================================================
void HBot::SetPosStraight( int x, int y )
{  
  //log  
//  Serial.println("SetPosStraight: x = ");
//  Serial.print( x );
//  Serial.print(", y = ");
//  Serial.println( y ); Serial.println("");
  //========================
  
  SetPosInternal( x, y ); // set m_GoalStep for M1 & M2
  UpdatePosStraight(); // use algorithm to calculate goal speed and set m_GoalSpeed for M1 & M2
} // SetPosStraight
