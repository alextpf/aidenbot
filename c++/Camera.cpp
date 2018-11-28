#include "Camera.h"
#include "../arduino/aidenbot/Configuration.h"
#include <iostream>

//=========================================================
Camera::Camera()
    : m_PredictXAttack( 0 )
    , m_PredictStatus( NO_RISK )
    , m_CurrNumPredictBounce( 0 )
	, m_PrevNumPredictBounce( 0 )
    , m_PredictTimeDefence( 0 )
    , m_PredictTimeAttack( 0 )
{}

//=========================================================
Camera::~Camera()
{}

//=========================================================
bool Camera::ToCorrectStep( int dt /*ms*/ )
{
	cv::Point posDif = m_CurrBotPos - m_PrevBotPos;

	m_PrevBotSpeed = m_CurrBotSpeed; // update old speed

	const cv::Point2f tmp = static_cast<cv::Point2f>( posDif * 100 );
	m_CurrBotSpeed = tmp / dt; // speed in dm/ms (we use this units to not overflow the variable)

	// if the bot is not moving fast (i.e. the detection is more accurate )
	// we feed the position back to arduino
	int speedThresh = 30;

	return (
		std::abs( m_CurrBotSpeed.x ) < speedThresh &&
		std::abs( m_CurrBotSpeed.y ) < speedThresh &&
		std::abs( m_PrevBotSpeed.x ) < speedThresh &&
		std::abs( m_PrevBotSpeed.y ) < speedThresh );
}

//=========================================================
void Camera::CamProcess( int dt /*ms*/ )
{
	// Speed calculation on each axis
	cv::Point posDif = m_CurrPuckPos - m_PrevPuckPos;

	m_PrevPuckSpeed = m_CurrPuckSpeed; // update old speed
	const cv::Point2f tmp = static_cast<cv::Point2f>( posDif * 100 );
	m_CurrPuckSpeed = tmp / dt; // speed in dm/ms (we use this units to not overflow the variable)

	m_BouncePos.x = -1;
	m_BouncePos.y = -1;

	// Noise detection, if there are a big speeds this should be noise
	if ( m_CurrPuckSpeed.x < -1000 ||
		m_CurrPuckSpeed.x >  1000 ||
		m_CurrPuckSpeed.y < -1000 ||
		m_CurrPuckSpeed.y >  1000 )
	{
		//std::cout << "NOISE" << std::endl;
		m_PredictStatus = ERROR;
		m_PrevPredictPos.x = -1;

		return;
	}

	cv::Point2f speedDif( std::abs( m_CurrPuckSpeed.x - m_PrevPuckSpeed.x ), std::abs( m_CurrPuckSpeed.y - m_PrevPuckSpeed.y ) );

	float SPEED_THRESH = 50.0f;

	if ( speedDif.x < SPEED_THRESH || speedDif.y < SPEED_THRESH )
	{
		m_AverageSpeed = m_CurrPuckSpeed * 0.7f + m_PrevPuckSpeed * 0.3f;
	}
	else
	{
		m_AverageSpeed = m_CurrPuckSpeed;
	}

	m_PredictXAttack = -1;

	// It's time to predict...
	// Based on current & previous position we predict the future
	// Posible impact? speed Y is negative when the puck is moving to the robot
	if ( m_AverageSpeed.y < FAST_IN_Y_SPEED )
	{
		// Puck is comming...
		// We need to predict the puck position when it reaches our goal Y position = defense_position
		// slope formula: m = (y2-y1)/(x2-x1)
		float slope = posDif.x == 0 ?   // To avoid division by 0
			9999999.0f : static_cast<float>( posDif.y ) / static_cast<float>( posDif.x );

		// Prediction of the new x position at defense position: x2 = (y2-y1)/m + x1
		m_CurrPredictPos.y = ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE;
		m_CurrPredictPos.x = static_cast<int>( static_cast<float>( m_CurrPredictPos.y - m_CurrPuckPos.y ) / slope ) + m_CurrPuckPos.x;

		// Prediction of the new x position at attack position
		m_PredictXAttack = m_CurrPredictPos.x;

		// puck has a bounce with side wall?
		const bool hasBounce = HasBounce( m_CurrPredictPos.x );
		if ( hasBounce )
		{
			m_PredictStatus = ONE_BOUNCE;

			// We start a new prediction

			// Wich side?
			m_BouncePos.x = m_CurrPredictPos.x < PUCK_SIZE ?
				PUCK_SIZE /*Left side*/ : TABLE_WIDTH - PUCK_SIZE /*Right side*/;

			m_BouncePos.y = static_cast<int>( static_cast<float>( m_BouncePos.x - m_CurrPuckPos.x ) * slope ) + m_CurrPuckPos.y;

			m_PredictTimeDefence = static_cast<int>( static_cast<float>( m_BouncePos.y - m_CurrPuckPos.y ) * 100.0f / m_CurrPuckSpeed.y ); // time until bouce
																																					   // bounce prediction => slope change  with the bounce, we only need to change the sign, easy!!
			slope = -slope;

			m_CurrPredictPos.y = ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE;
			m_CurrPredictPos.x = static_cast<int>( static_cast<float>( m_CurrPredictPos.y - m_BouncePos.y ) / slope ) + m_BouncePos.x;

			const bool hasAnotherBounce = HasBounce( m_CurrPredictPos.x );
			if ( hasAnotherBounce ) // New bounce with side wall?
			{
				m_CurrNumPredictBounce = 2;
				// We do nothing then... with two bounces there are small risk of goal...
				m_PrevPredictPos.x = -1;
				m_PredictStatus = NO_RISK; // no risk
			}
			else
			{
				m_CurrNumPredictBounce = 1;

				// only one side bounce...
				// If the puckSpeedY has changed a lot this mean that the puck has touched one side
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
					m_PredictTimeDefence += static_cast<int>( ( m_CurrPredictPos.y - m_BouncePos.y ) * 120.0f / m_CurrPuckSpeed.y ); // in ms
					m_PredictTimeDefence -= VISION_SYSTEM_LAG;
				}
			}
		} // puck has a bounce with side wall
		else
		{
			// No bounce, direct impact
			m_PredictStatus = DIRECT_IMPACT;
			m_CurrNumPredictBounce = 0;

			// ==1 or 2 means the first direct impact trajectory after a bounce.
			// We dont predict, because the previous pos is before bounce, so the calculated
			// offset & speed is not correct.
			if ( m_PrevNumPredictBounce == 0 )
			{
				// average of the results (some noise filtering)
				if ( m_PrevPredictPos.x > 0 )
				{
					m_CurrPredictPos.x = static_cast<int>( ( m_PrevPredictPos.x + m_CurrPredictPos.x ) * 0.5f );
				}

				m_PrevPredictPos.x = m_CurrPredictPos.x;

				m_PredictTimeDefence = static_cast<int>( ( ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE - m_CurrPuckPos.y ) * 100.0f / m_CurrPuckSpeed.y ) - VISION_SYSTEM_LAG; // in ms
				m_PredictTimeAttack = static_cast<int>( ( ROBOT_DEFENSE_ATTACK_POSITION_DEFAULT + PUCK_SIZE - m_CurrPuckPos.y ) * 100.0f / m_CurrPuckSpeed.y ) - VISION_SYSTEM_LAG; // in ms
			}//if ( m_PrevNumPredictBounce == 0 )
		} // // No bounce, direct impact
	}// coming fast into our field
	else
	{
		// Puck is moving slowly, or to the other side
		m_PrevPredictPos.x = -1;
		m_PredictStatus = NO_RISK;
	}//if ( m_AverageSpeed.y < -50 )

	m_PrevNumPredictBounce = m_CurrNumPredictBounce;
} // CamProcess

//=========================================================
bool Camera::HasBounce( const int x ) const
{
	return x < PUCK_SIZE || x > TABLE_WIDTH - PUCK_SIZE;
}

//=========================================================
cv::Point Camera::PredictPuckPos( int predictTime )
{
	predictTime += VISION_SYSTEM_LAG;
	cv::Point tmpPos( m_AverageSpeed * predictTime / 100.0f );
	return m_CurrPuckPos + tmpPos;
} // PredictPuckYPos

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
void Camera::SetCurrBotPos( const cv::Point& pos )
{
	m_CurrBotPos = pos;
}

//=========================================================
cv::Point Camera::GetCurrBotPos() const
{
	return m_CurrBotPos;
}

//=========================================================
void Camera::SetPrevBotPos( const cv::Point& pos )
{
	m_PrevBotPos = pos;
}

//=========================================================
cv::Point Camera::GetPrevBotPos() const
{
	return m_PrevBotPos;
}

//=========================================================
unsigned int Camera::GetCurrNumPredictBounce()
{
    return m_CurrNumPredictBounce;
}

//=========================================================
unsigned int Camera::GetPrevNumPredictBounce()
{
	return m_PrevNumPredictBounce;
}

//=========================================================
int Camera::GetPredictTimeDefence() const
{
    return m_PredictTimeDefence;
}

//=========================================================
int Camera::GetPredictTimeAttack() const
{
    return m_PredictTimeAttack;
}

//=========================================================
Camera::PREDICT_STATUS Camera::GetPredictStatus() const
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
cv::Point Camera::GetBouncePos() const
{
	return m_BouncePos;
}
