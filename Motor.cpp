//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#include "Motor.h"
#include "Configuration.h"

extern int freeRam ();
extern int myAbs(int param);
extern long myAbs(long param);
extern int sign(int val);

//=========================================================
Motor::Motor()
: m_CurrStep( 0 )
, m_GoalStep( 0 )
, m_Dir( 0 )
, m_Accel( 0 )
, m_CurrSpeed( 0 )
, m_GoalSpeed( 0 )
, m_MaxAbsSpeed( 0 )
, m_MaxAbsAccel( 0 )
, m_Period( 0 )
{}

//=========================================================
Motor::~Motor()
{}

//=========================================================
void Motor::UpdateAccel()
{  
  m_Accel = m_MaxAbsAccel;
  
  int absSpeed = abs( m_CurrSpeed );

  if( absSpeed < SCURVE_LOW_SPEED )
  {
    m_Accel = map( absSpeed, 0, SCURVE_LOW_SPEED, MIN_ACCEL, m_MaxAbsAccel ); 
  }

  m_Accel *= sign(m_GoalSpeed);

  #ifdef SHOW_LOG
      //log...
      Serial.println("Motor::UpdateAccel: ");
      Serial.print("absSpeed = ");
      Serial.println(absSpeed);
      Serial.print("m_Accel = ");
      Serial.println(m_Accel);
      //===========================
  #endif
  
} // UpdateAccel

//=========================================================
void Motor::UpdateSpeed( uint16_t dt, MOTOR_NUM m )
{
  const int tmp = m_Accel == 0 ? 0 : (long)m_CurrSpeed * m_CurrSpeed / ( STOP_COEF * (long)m_Accel );
  const int stopPos = m_CurrStep + tmp;

  #ifdef SHOW_LOG
      // log
      Serial.print( "m_CurrStep= " );
      Serial.println( m_CurrStep );
      Serial.print( "m_CurrSpeed= " );
      Serial.println( m_CurrSpeed );
      Serial.print( "tmp= " );
      Serial.println( tmp );
      //================================
  #endif
      
  int goalSpeed = 0;
  
  if( m_GoalStep > m_CurrStep ) // Positive move
  {
    #ifdef SHOW_LOG
                //log...
                Serial.println( "m_GoalStep > m_CurrStep" );
                Serial.print( "m_GoalStep= " );
                Serial.println( m_GoalStep );
                Serial.print( "m_CurrStep= " );
                Serial.println( m_CurrStep );
                
                Serial.print( "stopPos= " );
                Serial.println( stopPos );
                //=============================
    #endif
    
    // Start decelerating ?
    if ( stopPos >= m_GoalStep )
    {          
      goalSpeed = 0;

      #ifdef SHOW_LOG
          //log...
          Serial.println( "Postive move. Start deceleration1: stopPos >= m_GoalStep" );
          Serial.println( "goalSpeed = " );
          Serial.println( goalSpeed );
          //================================
     #endif
     
    }
    else if ( stopPos < m_GoalStep )
    {          
      goalSpeed = abs( m_GoalSpeed );

      #ifdef SHOW_LOG
          //log...
          Serial.println( "Postive move. Start deceleration2: stopPos < m_GoalStep" );
          Serial.println( "goalSpeed = " );
          Serial.println( goalSpeed );
          //================================
      #endif
      
    }        
  }
  else // negative move
  {
    #ifdef SHOW_LOG
                //log...  
                Serial.println( "m_GoalStep <= m_CurrStep" );
                Serial.print( "m_GoalStep= " );
                Serial.println( m_GoalStep );
                Serial.print( "m_CurrStep= " );
                Serial.println( m_CurrStep );
                
                Serial.print( "stopPos= " );
                Serial.println( stopPos );
                //=============================
    #endif
    
//    goalSpeed = stopPos <= m_GoalStep ? 0 : m_GoalSpeed;
        
    if ( stopPos <= m_GoalStep )
    {              
      goalSpeed = 0;

      #ifdef SHOW_LOG
        //log...
        Serial.println( "Negative move. Start deceleration1: stopPos <= m_GoalStep" );
        Serial.println( "goalSpeed = " );
        Serial.println( goalSpeed );
        //===========================
      #endif
    }
    else if ( stopPos > m_GoalStep )
    {        
      goalSpeed = -abs( m_GoalSpeed );

      #ifdef SHOW_LOG
        //log...
        Serial.println( "Negative move. Start deceleration2: stopPos > m_GoalStep" );
        Serial.println( "goalSpeed = " );
        Serial.println( goalSpeed );
        //===========================
      #endif
    }
  }
  
  m_GoalSpeed = sign( goalSpeed ) * abs( m_GoalSpeed );
  m_Accel = sign( goalSpeed ) * abs( m_Accel ); // direction may change

  #ifdef SHOW_LOG
        // log  
        Serial.print( "m_GoalSpeed = " );
        Serial.println( m_GoalSpeed );
        //===============================
  #endif
  
  SetCurrSpeedInternal( dt, goalSpeed, m );
} // UpdateSpeed

//=========================================================
void Motor::SetCurrSpeedInternal( uint16_t dt, int goalSpeed, MOTOR_NUM m )
{
  goalSpeed = constrain( goalSpeed, -MAX_ABS_SPEED, MAX_ABS_SPEED );
  
  // We limit acceleration => speed ramp
  int accel = ((long)m_Accel * dt) / 1000; // We divide by 1000 because dt are in microseconds

  #ifdef SHOW_LOG
      // log
      Serial.print( "dt = " );
      Serial.println( dt );
      Serial.print( "m_Accel = " );
      Serial.println( m_Accel );
      Serial.print( "accel = " );
      Serial.println( accel );
      // ==========================
  #endif
        
  int speedDif = goalSpeed - m_CurrSpeed;
  
  if ( abs( speedDif ) > abs( accel ) ) // We use long here to avoid overflow on the operation
  { 
    m_CurrSpeed += accel;
    
    #ifdef SHOW_LOG
      Serial.println( "m_CurrSpeed += accel= " );
      Serial.println( m_CurrSpeed );
    #endif
  }
  else
  {
    m_CurrSpeed = goalSpeed;

    #ifdef SHOW_LOG
        //log...
        Serial.println( "SetCurrSpeedInternal: 3rd clause " );
        Serial.println( "m_CurrSpeed = " );
        Serial.println( m_CurrSpeed );
        //===========================
    #endif
    
  }  
  
  // Check if we need to change the direction pins
  if ( m_CurrSpeed == 0 && m_Dir != 0 )
  {
    m_Dir = 0;
  }
  else if ( m_CurrSpeed > 0 && m_Dir != 1 )
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
  else if ( m_CurrSpeed < 0 && m_Dir != -1 )
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

  #ifdef SHOW_LOG
      //log...
        Serial.print("m_CurrSpeed =  ");
        Serial.println(m_CurrSpeed);
        Serial.print("m_Dir =  ");
        Serial.println(m_Dir);
      //===========================
  #endif
      
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
    // Check  if we need to reset the timer...
    if ( TCNT1 > OCR1A )
    {
      TCNT1 = 0;
    }
  }
  else if ( m == M2 )
  {
    OCR3A = m_Period;
    // Check  if we need to reset the timer...
    if ( TCNT3 > OCR3A )
    {
      TCNT3 = 0;
    }
  }  
  
} // SetCurrSpeedInternal
