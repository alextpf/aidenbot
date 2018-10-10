#include "Camera.h"
#include "Configuration.h"
#include <iostream>

//=========================================================
Camera::Camera()
: m_PredictXAttack( 0 )
, m_PredictStatus( 0 )
, m_NumPredictBounce( 0 )
, m_PredictBounceStatus( 0 )
, m_PredictTime( 0 )
, m_PredictTimeAttack( 0 )
{}

//=========================================================
Camera::~Camera()
{}

//=========================================================
void Camera::SetCurrPuckPos( const cv::Point& pos )
{
	m_CurrPuckPos = pos;
}

//=========================================================
cv::Point Camera::GetCurrPuckPos() const
{
	return m_CurrPuckPos;
}

//=========================================================
void Camera::SetPrevPuckPos( const cv::Point& pos )
{
	m_PrevPuckPos = pos;
}

//=========================================================
cv::Point Camera::GetPrevPuckPos() const
{
	return m_PrevPuckPos;
}

//=========================================================
int Camera::GetNumPredictBounce()
{
	return m_NumPredictBounce;
}

//=========================================================
int Camera::GetPredictTime() const
{
	return m_PredictTime;
}

//=========================================================
int Camera::GetPredictTimeAttack() const
{
	return m_PredictTimeAttack;
}

//=========================================================
int Camera::GetPredictStatus() const
{
	return m_PredictStatus;
}

//=========================================================
cv::Point Camera::GetCurrPredictPos() const
{
	return m_CurrPredictPos;
}

//=========================================================
cv::Point2f Camera::GetPuckAvgSpeed() const
{
	return m_AverageSpeed;
}

//=========================================================
cv::Point2f Camera::GetCurrPuckSpeed() const
{
	return m_CurrPuckSpeed;
}

//=========================================================
int Camera::GetPredictXAttack()
{
	return m_PredictXAttack;
}

//=========================================================
void Camera::SetCurrPredictPos( const cv::Point& pos )
{
	m_CurrPredictPos = pos;
}

//=========================================================
void Camera::CamProcess( int dt /*ms*/ )
{  
  // Speed calculation on each axis  
  cv::Point vec = m_CurrPuckPos - m_PrevPuckPos;

  m_PrevPuckSpeed = m_CurrPuckSpeed; // update old speed
  m_CurrPuckSpeed = vec * 100 / dt; // speed in dm/ms (we use this units to not overflow the variable)
  
  // Noise detection, if there are a big speeds this should be noise
  if ( m_CurrPuckSpeed.x < -1000 || 
       m_CurrPuckSpeed.x >  1000 || 
       m_CurrPuckSpeed.y < -1000 || 
       m_CurrPuckSpeed.y >  1000 )
  {
    std::cout << "NOISE" << std::endl;
    m_PredictStatus = -1;
    m_PrevPredictPos.x = -1;

    return;
  }
  
  if ( m_PredictStatus == -1 )  // Noise on last?
  {
    m_AverageSpeed = m_CurrPuckSpeed;
  }
  else
  {
    // if there are low accelerations (similar speeds on readings) we apply an average filtering with the previous value...
    m_AverageSpeed.x = std::abs( m_CurrPuckSpeed.x - m_PrevPuckSpeed.x ) < 50 ?
      ( m_CurrPuckSpeed.x + m_PrevPuckSpeed.x ) * 0.5f : m_CurrPuckSpeed.x;
      
    m_AverageSpeed.y = std::abs( m_CurrPuckSpeed.y - m_PrevPuckSpeed.y ) < 50 ?
      ( m_CurrPuckSpeed.y + m_PrevPuckSpeed.y ) * 0.5f : m_CurrPuckSpeed.y;
  }

  m_PredictXAttack = -1;
  
  // It´s time to predict...
  // Based on actual position and move vector we need to know the future...
  // Posible impact? speed Y is negative when the puck is moving to the robot
  if ( m_AverageSpeed.y < -50 )  //-25
  {
    m_PredictStatus = 1;
    
    // Puck is comming...
    // We need to predict the puck position when it reaches our goal Y position = defense_position
    // slope formula: m = (y2-y1)/(x2-x1)
    float slope = vec.x == 0 ?   // To avoid division by 0
		9999999.0f : static_cast<float>( vec.y ) / static_cast<float>( vec.x );
    
    // Prediction of the new x position at defense position: x2 = (y2-y1)/m + x1
    m_CurrPredictPos.y = ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE;
	m_CurrPredictPos.x = static_cast<int>( static_cast<float>( m_CurrPredictPos.y - m_CurrPuckPos.y ) / slope ) + m_CurrPuckPos.x;
    
    // Prediction of the new x position at attack position
	m_PredictXAttack = static_cast<int>( static_cast<float>( ROBOT_DEFENSE_ATTACK_POSITION_DEFAULT + PUCK_SIZE - m_CurrPuckPos.y ) / slope ) + m_CurrPuckPos.x;

    // puck has a bounce with side wall?
    if ( m_CurrPredictPos.x < PUCK_SIZE || 
         m_CurrPredictPos.x > TABLE_WIDTH - PUCK_SIZE )
    {
      m_PredictStatus = 2;
      m_NumPredictBounce = 1;
      m_PredictBounceStatus = 1;
      
      // We start a new prediction

      cv::Point bouncePos;
      
      // Wich side?
      bouncePos.x = m_CurrPredictPos.x < PUCK_SIZE ?
          PUCK_SIZE /*Left side*/: TABLE_WIDTH - PUCK_SIZE /*Right side*/;
          
	  bouncePos.y = static_cast<int>( static_cast<float>( bouncePos.x - m_CurrPuckPos.x ) * slope ) + m_CurrPuckPos.y;
      
	  m_PredictTime = static_cast<int>( static_cast<float>( bouncePos.y - m_CurrPuckPos.y ) * 100.0f / m_CurrPuckSpeed.y ); // time until bouce
      
      // bounce prediction => slope change  with the bounce, we only need to change the sign, easy!!
      slope = -slope;
      
      m_CurrPredictPos.y = ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE;
	  m_CurrPredictPos.x = static_cast<int>( static_cast<float>( m_CurrPredictPos.y - bouncePos.y ) / slope ) + bouncePos.x;
      
      if ( m_CurrPredictPos.x < PUCK_SIZE || 
           m_CurrPredictPos.x > TABLE_WIDTH - PUCK_SIZE ) // New bounce with side wall?
      {
        // We do nothing then... with two bounces there are small risk of goal...
        m_PrevPredictPos.x = -1;
        m_PredictStatus = 0; // no risk
      }
      else
      {
        // only one side bounce...
        // If the puckSpeedY has changed a lot this mean that the puck has touch one side
        if ( std::abs( m_CurrPuckSpeed.y - m_PrevPuckSpeed.y ) > 50 )
        {
          // We dont make a new prediction...
          m_PrevPredictPos.x = -1;
        }
        else
        {
          // average of the results (some noise filtering)
          if ( m_PrevPredictPos.x != -1 )
          {
			  m_CurrPredictPos.x = static_cast<int>( ( m_PrevPredictPos.x + m_CurrPredictPos.x ) * 0.5f );
          }
          
          m_PrevPredictPos.x = m_CurrPredictPos.x;
          
          // We introduce a factor (120 instead of 100) to model the bounce (20% loss in speed)(to improcve...)
          m_PredictTime += static_cast<int>( ( m_CurrPredictPos.y - m_CurrPuckPos.y ) * 120.0f / m_CurrPuckSpeed.y ); // in ms
          m_PredictTime -= VISION_SYSTEM_LAG;
        }
      }
    }
    else // No bounce, direct impact
    {
      if (m_PredictBounceStatus == 1)  // This is the first direct impact trajectory after a bounce
      {
        // We dont predict nothing new...
        m_PredictBounceStatus = 0;
      }
      else
      {
        // average of the results (some noise filtering)
        if ( m_PrevPredictPos.x > 0 )
        {
			m_CurrPredictPos.x = static_cast<int>( ( m_PrevPredictPos.x + m_CurrPredictPos.x ) * 0.5f );
        }
        m_PrevPredictPos.x = m_CurrPredictPos.x;

		m_PredictTime = static_cast<int>( ( ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE - m_CurrPuckPos.y ) * 100.0f / m_CurrPuckSpeed.y ) - VISION_SYSTEM_LAG; // in ms
		m_PredictTimeAttack = static_cast<int>( ( ROBOT_DEFENSE_ATTACK_POSITION_DEFAULT + PUCK_SIZE - m_CurrPuckPos.y ) * 100.0f / m_CurrPuckSpeed.y ) - VISION_SYSTEM_LAG; // in ms        
      }
    }
  }
  else // // Puck is moving slowly or to the other side
  {
    m_PrevPredictPos.x = -1;
    m_PredictStatus = 0;
    m_NumPredictBounce = 0;
    m_PredictBounceStatus = 0;
  }//if ( m_AverageSpeed.y < -50 )
} // CamProcess

//=========================================================
cv::Point Camera::PredictPuckPos( int predictTime )
{
  predictTime += VISION_SYSTEM_LAG;
  cv::Point tmpPos( m_AverageSpeed * m_PredictTime / 100.0f );
  return m_CurrPuckPos + tmpPos;
} // PredictPuckYPos
