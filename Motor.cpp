//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#include "Motor.h"
#include "Configuration.h"
//#include "Util.h"

extern int freeRam ();
extern int myAbs(int param);
extern int sign(int val);

//=========================================================
void Motor::UpdateAccel()
{  
  int absSpeed = abs( m_Speed );

  if( absSpeed < SCURVE_LOW_SPEED )
  {
    m_Accel = map( absSpeed, 0, SCURVE_LOW_SPEED, MIN_ACCEL, m_MaxAccel );  
    if( m_Accel > m_MaxAccel )
    {
      m_Accel = m_MaxAccel;
    }
  }
}

//=========================================================
void Motor::UpdateSpeed( int dt )
{
  int tmp = m_Speed * m_Speed / ( 1800.0 * m_Accel );
  int stopPos = m_Step + sign(m_Speed) * tmp;

  int goalSpeed;
  
  if( m_GoalStep > m_Step ) // Positive move
  {
    // Start decelerating ?
    goalSpeed = stopPos >= m_GoalStep ? 0 : m_GoalSpeed;
  }
  else // negative move
  {
    goalSpeed = stopPos <= m_GoalStep ? 0 : -m_GoalSpeed;
  }

  SetSpeedInternal( dt, goalSpeed );
} // UpdateSpeed

//=========================================================
void Motor::SetSpeedInternal( int dt, int goalSpeed )
{
  goalSpeed = constrain( goalSpeed, -m_MaxSpeed, m_MaxSpeed );
  
  // We limit acceleration => speed ramp
  int accel = (long)m_Accel * dt * 0.001; // We divide by 1000 because dt are in microseconds
  if ( (long)goalSpeed - m_Speed > accel ) // We use long here to avoid overflow on the operation
  { 
    m_Speed += accel;
  }
  else if ( (long)m_Speed - goalSpeed > accel)
  {
    m_Speed -= accel;
  }
  else
  {
    m_Speed = goalSpeed;
  }  
  
  // Check if we need to change the direction pins
  if ( (m_Speed == 0) && (m_Dir != 0) )
  {
    m_Dir = 0;
  }
  else if ( (m_Speed > 0) && (m_Dir != 1) )
  {
    m_Dir = 1;
  }
  else if ((m_Speed < 0) && (m_Dir != -1))
  {
    m_Dir = -1;
  }

  if (m_Speed == 0)
  {
    m_Period = ZERO_SPEED;
  }
  else if (m_Speed > 0)
  {
    m_Period = 2000000 / m_Speed; // 2Mhz timer
  }
  else
  {
    m_Period = 2000000 / -m_Speed;
  }

  if (m_Period > 65535)   // Check for minimun speed (maximun period without overflow)
  {
    m_Period = ZERO_SPEED;
  }
  
} // SetSpeed
