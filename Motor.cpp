//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#include "Motor.h"
#include "Configuration.h"

extern int freeRam ();
extern int myAbs(int param);
extern int sign(int val);

//=========================================================
Motor::Motor()
: m_CurrStep( 0 )
, m_GoalStep( 0 )
, m_Dir( 0 )
, m_Accel( 0 )
, m_CurrSpeed( 0 )
, m_GoalSpeed( 0 )
, m_MaxSpeed( 0 )
, m_MaxAccel( 0 )
, m_Period( 0 )
{}

//=========================================================
Motor::~Motor()
{}

//=========================================================
void Motor::UpdateAccel()
{  
  int absSpeed = abs( m_CurrSpeed );

  if( absSpeed < SCURVE_LOW_SPEED )
  {
    m_Accel = map( absSpeed, 0, SCURVE_LOW_SPEED, MIN_ACCEL, m_MaxAccel );  
    if( m_Accel > m_MaxAccel )
    {
      m_Accel = m_MaxAccel;
    }
  }
  //log...
//  Serial.println("Motor::UpdateAccel: ");
//  Serial.print("absSpeed = ");
//  Serial.println(absSpeed);
//  Serial.print("m_Accel = ");
//  Serial.println(m_Accel);
  //===========================
} // UpdateAccel

//=========================================================
void Motor::UpdateSpeed( int dt, MOTOR_NUM m )
{
  int tmp = m_CurrSpeed * m_CurrSpeed / ( 1800.0 * m_Accel );
  int stopPos = m_CurrStep + sign(m_CurrSpeed) * tmp;

  int goalSpeed;
  
  if( m_GoalStep > m_CurrStep ) // Positive move
  {
    // Start decelerating ?
    goalSpeed = stopPos >= m_GoalStep ? 0 : m_GoalSpeed;
  }
  else // negative move
  {
    goalSpeed = stopPos <= m_GoalStep ? 0 : -m_GoalSpeed;
  }

  SetCurrSpeedInternal( dt, goalSpeed, m );
} // UpdateSpeed

//=========================================================
void Motor::SetCurrSpeedInternal( int dt, int goalSpeed, MOTOR_NUM m )
{
  goalSpeed = constrain( goalSpeed, -MAX_SPEED, MAX_SPEED );
  
  // We limit acceleration => speed ramp
  int accel = (long)m_Accel * dt * 0.001; // We divide by 1000 because dt are in microseconds
  if ( (long)goalSpeed - m_CurrSpeed > accel ) // We use long here to avoid overflow on the operation
  { 
    m_CurrSpeed += accel;
  }
  else if ( (long)m_CurrSpeed - goalSpeed > accel)
  {
    m_CurrSpeed -= accel;
  }
  else
  {
    m_CurrSpeed = goalSpeed;
  }  
  
  // Check if we need to change the direction pins
  if ( (m_CurrSpeed == 0) && (m_Dir != 0) )
  {
    m_Dir = 0;
  }
  else if ( (m_CurrSpeed > 0) && (m_Dir != 1) )
  {
    m_Dir = 1;
    if ( m == M1 )
    {
      SET(PORTF,1);
    }
    else if ( m == M2 )
    {
      SET(PORTF,7);
    }
  }
  else if ((m_CurrSpeed < 0) && (m_Dir != -1))
  {
    m_Dir = -1;
    if ( m == M1 )
    {
      CLR(PORTF,1);
    }
    else if ( m == M2 )
    {
      CLR(PORTF,7);
    }
  }
  
  //log...
//    Serial.print("m_CurrSpeed =  ");
//    Serial.println(m_CurrSpeed);
//    Serial.print("m_Dir =  ");
//    Serial.println(m_Dir);
  //===========================
  
  if (m_CurrSpeed == 0)
  {
    m_Period = ZERO_SPEED;
  }
  else if (m_CurrSpeed > 0)
  {
    m_Period = 2000000 / m_CurrSpeed; // 2Mhz timer
  }
  else
  {
    m_Period = 2000000 / -m_CurrSpeed;
  }

  if (m_Period > 65535)   // Check for minimun speed (maximun period without overflow)
  {
    m_Period = ZERO_SPEED;
  }
  //log...
//    Serial.print("m_Period =  ");
//    Serial.println(m_Period);
  //===========================
  if ( m == M1 )
  {
    OCR1A = m_Period;
  }
  else if ( m == M2 )
  {
    OCR3A = m_Period;
  }
  
} // SetCurrSpeedInternal
