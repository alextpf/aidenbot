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
void Motor::UpdateSpeed()
{
  int tmp = m_Speed * m_Speed / ( 1800.0 * m_Accel );
  int stopPos = m_Step + sign(m_Speed) * tmp;

  if( m_GoalStep > m_Step ) // Positive move
  {
    // Start decelerating ?
    m_Speed = stopPos >= m_GoalStep ? 0 : m_GoalSpeed;
  }
  else // negative move
  {
    m_Speed = stopPos <= m_GoalStep ? 0 : -m_GoalSpeed;
  }
}
