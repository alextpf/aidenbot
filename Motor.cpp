#include "Motor.h"
#include "Configuration.h"
#include "Util.h"

//=========================================================
void Motor::UpdateAccel()
{  
  int16_t absSpeed = abs( m_Speed );

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
void Motor::UpdateSpeed( int16_t dt )
{
  int tmp = m_Speed * m_Speed / ( 1800.0 * m_Accel );
  int stopPos = m_Step + sign(m_Speed) * tmp;

  int16_t goalSpeed;
  
  if( m_GoalStep > m_Step ) // Positive move
  {
    // Start decelerating ?
    goalSpeed = stopPos >= m_GoalStep ? 0 : m_GoalSpeed;
  }
  else // negative move
  {
    goalSpeed = stopPos <= m_GoalStep ? 0 : -m_GoalSpeed;
  }

  SetSpeed( dt, goalSpeed );
} // UpdateSpeed

//=========================================================
void Motor::SetSpeed( int16_t dt, int16_t goalSpeed )
{
  goalSpeed = constrain( goalSpeed, -m_MaxSpeed, m_MaxSpeed );
  
  // We limit acceleration => speed ramp
  int16_t accel = (long)m_Accel * dt * 0.001; // We divide by 1000 because dt are in microseconds
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
} // SetSpeed
