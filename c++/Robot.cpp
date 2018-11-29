//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#include "Robot.h"
#include "../arduino/aidenbot/Configuration.h"
#include <iostream>

//====================================================================================================================
Robot::Robot()
	: m_RobotStatus( BOT_STATUS::INIT )
	, m_AttackTime( 0 )
	, m_AttackStatus( ATTACK_STATUS::WAIT_FOR_ATTACK )
    , m_DesiredYSpeed( 0 )
    , m_DesiredXSpeed( 0 )
{}

//====================================================================================================================
Robot::~Robot()
{}

//====================================================================================================================
int Robot::GetDesiredRobotYSpeed()
{
	return m_DesiredYSpeed;
}

//====================================================================================================================
int Robot::GetDesiredRobotXSpeed()
{
    return m_DesiredXSpeed;
}

//====================================================================================================================
cv::Point Robot::GetDesiredRobotPos()
{
    return m_DesiredRobotPos;
} // GetDesiredRobotPos

//====================================================================================================================
void Robot::NewDataStrategy( Camera& cam )
{
	m_RobotStatus = BOT_STATUS::INIT; // Going to initial position (defense)

    switch( cam.GetPredictStatus() )
    {
        case Camera::PREDICT_STATUS::OWN_GOAL:
            {
                m_RobotStatus = BOT_STATUS::TURN_AROUND;
            }
            break;

        case Camera::PREDICT_STATUS::NO_RISK:
            {
                // If the puck is moving slowly in the robot field we could start an attack
                if( cam.GetCurrPuckPos().y < ROBOT_CENTER_Y - 20 &&
                    std::abs( cam.GetCurrPuckSpeed().y ) < MIN_ABS_Y_SPEED &&
                    std::abs( cam.GetCurrPuckSpeed().x ) < MIN_ABS_X_SPEED )
                {
                    m_RobotStatus = BOT_STATUS::ATTACK;
                }
            }
            break;

        case Camera::PREDICT_STATUS::DIRECT_IMPACT:
            {
				const unsigned int prevNumPredictBounce = cam.GetPrevNumPredictBounce();

				if ( prevNumPredictBounce == 0 )
				{
					// Puck is moving to our field directly, with no bounce; Direct impact
					cv::Point predictPos = cam.GetCurrPredictPos();

					if ( ( predictPos.x > 0 /*ROBOT_MIN_X + PUCK_SIZE * 2*/ ) &&
					 	 ( predictPos.x < TABLE_WIDTH /*ROBOT_MAX_X - PUCK_SIZE * 2*/ ) )
					{
						// Predicted position X is within table range
						if ( cam.GetPuckAvgSpeed().y > MIN_PUCK_Y_SPEED1 )
						{
							// puck is moving not so fast
							m_RobotStatus = BOT_STATUS::DEFENCE_AND_ATTACK;
						}
						else
						{
							// puck is moving fast
                            m_RobotStatus = BOT_STATUS::DEFENCE;
						}
					}
					else
					{
						// Predicted position X is out of table range
						if ( cam.GetPredictTimeDefence() < BOUNCE_TIME_THRESHOLD ||
                             cam.GetPredictTimeAtBounce() < BOUNCE_TIME_THRESHOLD )
						{
							// predicted hit time is small, i.e. puck approcahing soon
							m_RobotStatus = BOT_STATUS::DEFENCE;
						}
						else
						{
							// predicted hit time is large, attack at the bounce point
							m_RobotStatus = BOT_STATUS::ATTACK_AT_BOUNCE;
						}
					}
				}
				else// if ( prevNumPredictBounce == 1 )
				{
					// puck coming from a bounce, this is the first frame after the bounce
					if ( cam.GetPuckAvgSpeed().y > MIN_PUCK_Y_SPEED2 )
					{
						// puck is moving not so fast
						m_RobotStatus = BOT_STATUS::DEFENCE_AND_ATTACK;
					}
					else
					{
						// puck is moving fast
						m_RobotStatus = BOT_STATUS::DEFENCE;
					}
				}
            }
            break;

        case Camera::PREDICT_STATUS::ONE_BOUNCE:
            {
                // Prediction with side bounce
                if( cam.GetPredictTimeDefence() < BOUNCE_TIME_THRESHOLD ||
                    cam.GetPredictTimeAtBounce() < BOUNCE_TIME_THRESHOLD )
                {
                    // Limit movement
                    m_RobotStatus = BOT_STATUS::DEFENCE;
                }
                else
                {
                    m_RobotStatus = BOT_STATUS::ATTACK_AT_BOUNCE;
                }
            }
            break;
        default:
            break;
    } // switch

} // Robot::NewDataStrategy

//====================================================================================================================
void Robot::RobotMoveDecision( Camera& cam )
{
    m_DesiredYSpeed = MAX_Y_ABS_SPEED;
    m_DesiredXSpeed = MAX_X_ABS_SPEED;

    cv::Point robotPos;

	switch ( m_RobotStatus )
	{
	case BOT_STATUS::INIT: // Go to init position
	{
        m_DesiredRobotPos.x = ROBOT_CENTER_X;  //center X axis
        m_DesiredRobotPos.y = ROBOT_DEFENSE_POSITION_DEFAULT;
        m_DesiredYSpeed = static_cast<int>( MAX_Y_ABS_SPEED * 0.6667 ); // Return a bit more slowly...
        m_DesiredXSpeed = static_cast<int>( MAX_X_ABS_SPEED * 0.6667 ); // Return a bit more slowly...

		m_AttackTime = 0;
	}
	break;

    case BOT_STATUS::TURN_AROUND: // try to go around to the back of the puck ( since it's current at the back of the bot )
    {
        // move only if the speed of puck is small
        const cv::Point2f& puckSpeed = cam.GetCurrPuckSpeed();
        const float speedThresh = 50;

        if( abs( puckSpeed.x ) < speedThresh && abs( puckSpeed.y ) < speedThresh )
        {
            const int thresh = 3 * PUCK_SIZE;
            const cv::Point& currBotPos = cam.GetCurrBotPos();
            const cv::Point& currPuckPos = cam.GetCurrPuckPos();

            // if puck and bot don't overlap vertically, back up straight
            if( std::abs( currPuckPos.x - currBotPos.x ) < thresh )
            {
                m_DesiredRobotPos.x = currBotPos.x;
                m_DesiredRobotPos.y = currPuckPos.y - thresh;
            }
            else
            {
                //else, move the bot away from puck horizontally
                // depending on which side the puck is at currently, move the bot
                if( currPuckPos.x > ROBOT_CENTER_X )
                {
                    m_DesiredRobotPos.x = currPuckPos.x - thresh;
                    m_DesiredRobotPos.y = currBotPos.y;
                }
                else
                {
                    m_DesiredRobotPos.x = currPuckPos.x + thresh;
                    m_DesiredRobotPos.y = currBotPos.y;
                }
            }

            m_DesiredYSpeed = static_cast<int>( MAX_Y_ABS_SPEED * 0.7 );
            m_DesiredXSpeed = static_cast<int>( MAX_X_ABS_SPEED * 0.7 );
        } // if( abs( puckSpeed.x ) < speedThresh && abs( puckSpeed.y ) < speedThresh )
    }//TURN_AROUND
    break;

	case BOT_STATUS::DEFENCE: // Defense mode (only move on X axis on the defense line)
	{
		cv::Point pos = cam.GetCurrPredictPos();

		if ( pos.x < ROBOT_MIN_X )
		{
			pos.x = ROBOT_MIN_X;
		}
		else if ( pos.x > ROBOT_MAX_X )
		{
			pos.x = ROBOT_MAX_X;
		}

		cam.SetCurrPredictPos( pos );

		m_DesiredRobotPos.y = ROBOT_DEFENSE_POSITION_DEFAULT;
        m_DesiredRobotPos.x = pos.x;

		m_AttackTime = 0;
	}
	break;

	case BOT_STATUS::DEFENCE_AND_ATTACK:
	{
		if ( cam.GetPredictTimeAttack() < MIN_PREDICT_TIME ) // If time is less than 150ms we start the attack
		{
            m_DesiredRobotPos.y = ROBOT_DEFENSE_ATTACK_POSITION_DEFAULT + PUCK_SIZE * 4; // we need some override
            m_DesiredRobotPos.x = cam.GetPredictXAttack();
		}
		else      // Defense position
		{
            m_DesiredRobotPos.y = ROBOT_DEFENSE_POSITION_DEFAULT;
            m_DesiredRobotPos.x = cam.GetCurrPredictPos().x;  // predict_x_attack;

			m_AttackTime = 0;
		}
	}
	break;

	case BOT_STATUS::ATTACK:
	{
		cv::Point attackPredictPos;

		if ( m_AttackTime == 0 )
		{
            attackPredictPos = cam.PredictPuckPos( ATTACK_TIME_THRESHOLD );

            if( ( attackPredictPos.x > PUCK_SIZE ) &&
                ( attackPredictPos.x < TABLE_WIDTH - PUCK_SIZE ) &&
                ( attackPredictPos.y > PUCK_SIZE * 2 ) &&
                ( attackPredictPos.y < ROBOT_CENTER_Y - PUCK_SIZE * 4 ) )
            {
                m_AttackTime = clock() + static_cast<clock_t>( ATTACK_TIME_THRESHOLD * CLOCKS_PER_SEC / 1000.0f );  // Prepare an attack in 500ms

                                                                                                                    // Go to pre-attack position
                m_DesiredRobotPos.x = attackPredictPos.x;
                m_DesiredRobotPos.y = attackPredictPos.y - PUCK_SIZE * 4;
                m_DesiredYSpeed = static_cast<int>( MAX_Y_ABS_SPEED * 0.5 );
                m_DesiredXSpeed = static_cast<int>( MAX_X_ABS_SPEED * 0.5 );

                m_AttackStatus = ATTACK_STATUS::READY_TO_ATTACK;
            }
            else
            {
                m_AttackTime = 0;  // Continue waiting for the right attack moment...
                m_AttackStatus = ATTACK_STATUS::WAIT_FOR_ATTACK;

                // And go to defense position
                m_DesiredRobotPos.y = ROBOT_DEFENSE_POSITION_DEFAULT;
                m_DesiredRobotPos.x = ROBOT_CENTER_X;  //center X axis
                m_DesiredYSpeed = static_cast<int>( MAX_Y_ABS_SPEED * 0.6667 ); // Return a bit more slowly...
                m_DesiredXSpeed = static_cast<int>( MAX_X_ABS_SPEED * 0.6667 ); // Return a bit more slowly...
            }
		} // if ( m_AttackTime == 0 )
		else
		{
			if ( m_AttackStatus == ATTACK_STATUS::READY_TO_ATTACK )
			{
				// ready to attack
                const int impactTime = static_cast<int>( ( m_AttackTime - clock() ) * 1000.0f / CLOCKS_PER_SEC ); // in ms
                if( impactTime < IMPACT_TIME_THRESHOLD )
                {
                    // Attack movement
                    attackPredictPos = cam.PredictPuckPos( impactTime );
                    m_DesiredRobotPos.x = attackPredictPos.x;
                    m_DesiredRobotPos.y = attackPredictPos.y + PUCK_SIZE * 2;

                    m_AttackStatus = ATTACK_STATUS::AFTER_ATTACK;
                }
                else  // m_AttackStatus = ATTACK_STATUS::READY_TO_ATTACK but it's not the time to attack yet
                {
                    // Go to pre-attack position
                    attackPredictPos = cam.PredictPuckPos( ATTACK_TIME_THRESHOLD );

                    m_DesiredRobotPos.x = attackPredictPos.x;
                    m_DesiredRobotPos.y = attackPredictPos.y - PUCK_SIZE * 4;

                    m_DesiredYSpeed = static_cast<int>( MAX_Y_ABS_SPEED * 0.5 );
                    m_DesiredXSpeed = static_cast<int>( MAX_X_ABS_SPEED * 0.5 );
                }
			} // if (m_AttackStatus == ATTACK_STATUS::READY_TO_ATTACK)

			if ( m_AttackStatus == ATTACK_STATUS::AFTER_ATTACK )
			{
				// after firing attack
				int dt = static_cast<int>( ( clock() - m_AttackTime ) * 1000.0f / CLOCKS_PER_SEC ); // in ms

				if ( dt > 80 ) // Attack move is done? => Reset to defense position
				{
					//Serial.print( "RESET" );
					m_AttackTime = 0;
					m_RobotStatus = BOT_STATUS::INIT;
					m_AttackStatus = ATTACK_STATUS::WAIT_FOR_ATTACK;
				}
			} // if ( m_AttackStatus == ATTACK_STATUS::AFTER_ATTACK )
		} // if (m_AttackTime == 0)
	} // case BOT_STATUS::ATTACK:
	break;

	case BOT_STATUS::ATTACK_AT_BOUNCE: // The puck will come from a bounce
	{
        cv::Point bouncePos = cam.GetBouncePos();

        if( bouncePos.x < ROBOT_MIN_X )
        {
            bouncePos.x = ROBOT_MIN_X;
        }
        else if( bouncePos.x > ROBOT_MAX_X )
        {
            bouncePos.x = ROBOT_MAX_X;
        }

        //wait for the perfect timing to attack
        const int impactTime = cam.GetPredictTimeAtBounce();

        if( impactTime < IMPACT_TIME_THRESHOLD )
        {
            // Attack movement
            // attack at the bounce point

            m_DesiredRobotPos = bouncePos;
        }
        else  //it's not the time to attack yet
        {
            // go to pre-attack position
            m_DesiredRobotPos.x = bouncePos.x;
            m_DesiredRobotPos.y = bouncePos.y - PUCK_SIZE * 4;

            m_DesiredYSpeed = static_cast<int>( MAX_Y_ABS_SPEED * 0.5 );
            m_DesiredXSpeed = static_cast<int>( MAX_X_ABS_SPEED * 0.5 );
        }
	} // ATTACK_AT_BOUNCE
	break;

	default:
		// Default : go to defense position
        m_DesiredRobotPos.y = ROBOT_DEFENSE_POSITION_DEFAULT;
        m_DesiredRobotPos.x = ROBOT_CENTER_X;  //center X axis

		m_AttackTime = 0;
	}// switch
} // Robot::RobotMoveDecision

//====================================================================================================================
bool Robot::IsOwnGoal( const Camera& cam )
{
	const int botPosY  = cam.GetCurrBotPos().y;
	const int puckPosX = cam.GetCurrPuckPos().x;
	const int puckPosY = cam.GetCurrPuckPos().y;

	return ( /*botPosY  > ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE  &&*/  // robot in front of defence line
		puckPosY < botPosY && // puck behind robot
		puckPosX > 0 /*ROBOT_CENTER_X - PUCK_SIZE * 5*/ &&     // puck X within table range
		puckPosX < TABLE_WIDTH/*ROBOT_CENTER_X + PUCK_SIZE * 5*/ );
} // IsOwnGoal
