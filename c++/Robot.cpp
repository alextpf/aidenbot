//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#include "Robot.h"
#include "Configuration.h"
#include <iostream>

//====================================================================================================================
Robot::Robot()
	: m_RobotStatus( 0 )
	, m_AttackTime( 0 )
	, m_AttackStatus( 0 )
{}

//====================================================================================================================
Robot::~Robot()
{}

//====================================================================================================================
void Robot::NewDataStrategy( Camera& cam )
{
	m_RobotStatus = 0; // Going to initial position (defense)

	if ( cam.GetPredictStatus() == 1 ) // Puck comming?
	{
		if ( cam.GetNumPredictBounce() == 0 )
		{
			// Direct impact

			cv::Point predictPos = cam.GetCurrPredictPos();

			if ( ( predictPos.x > ROBOT_MIN_X + PUCK_SIZE * 2 ) &&
				 ( predictPos.x < ROBOT_MAX_X - PUCK_SIZE * 2 ) )
			{
				// Predicted position X is within table range        
				if ( cam.GetPuckAvgSpeed().y > MIN_PUCK_Y_SPEED1 )
				{
					m_RobotStatus = 2;  // defense+attack
				}
				else
				{
					m_RobotStatus = 1;  // Puck too fast => only defense
				}
			}
			else
			{
				if ( cam.GetPredictTimeDefence() < PREDICT_TIME_THRESHOLD )
				{
					// predicted hit time is small, i.e. puck approcahing soon
					m_RobotStatus = 1;  // Defense
				}
				else
				{
					// predicted hit time is large, i.e. no risk          
					m_RobotStatus = 0;
				}
			}
		}
		else
		{
			// Puck come from a bounce
			if ( cam.GetPuckAvgSpeed().y > MIN_PUCK_Y_SPEED2 ) // Puck is moving fast?
			{
				m_RobotStatus = 2;  // Defense+Attack
			}
			else
			{
				m_RobotStatus = 1;  // Defense (too fast...)
			}
		}
	} // if ( cam.GetPredictStatus() == 1 ) // Puck comming?

	// Prediction with side bound
	if ( cam.GetPredictStatus() == 2 )
	{
		if ( cam.GetPredictTimeDefence() < PREDICT_TIME_THRESHOLD )
		{
			// Limit movement      
			m_RobotStatus = 1; // only defense mode
		}
		else
		{
			m_RobotStatus = 0; // no risk
		}
	} // if ( cam.GetPredictStatus() == 2)

	// If the puck is moving slowly in the robot field we could start an attack
	if ( cam.GetPredictStatus() == 0 &&
		 cam.GetCurrPuckPos().y < ROBOT_CENTER_Y - 20 &&
		 std::abs( cam.GetCurrPuckSpeed().y ) < MIN_ABS_Y_SPEED &&
		 std::abs( cam.GetCurrPuckSpeed().x ) < MIN_ABS_X_SPEED )
	{
		m_RobotStatus = 3; // attack
	}
} // Robot::NewDataStrategy

//====================================================================================================================
void Robot::RobotStrategy( Camera& cam )
{
	//hBot.SetMaxAbsSpeed( MAX_ABS_SPEED ); // default to max robot speed and accel
	//hBot.SetMaxAbsAccel( MAX_ABS_ACCEL );

	int posX, posY;
	switch ( m_RobotStatus )
	{
	case 0: // Go to defense position
	{
		posY = ROBOT_DEFENSE_POSITION_DEFAULT;
		posX = ROBOT_CENTER_X;  //center X axis
		//const int s = MAX_ABS_SPEED * 0.6667; // Return a bit more slowly...
		//hBot.SetMaxAbsSpeed( s );
/*
		if ( CheckOwnGoal( hBot, cam ) )
		{
			posX = hBot.GetRobotPos().x;
			posY = hBot.GetRobotPos().y;
		}

		hBot.SetPosStraight( posX, posY );
*/
		m_AttackTime = 0;
	}
	break;

	case 1: // Defense mode (only move on X axis on the defense line)
	{
		cv::Point pos = cam.GetCurrPredictPos();
		const int maxX = TABLE_WIDTH - PUCK_SIZE * 3;
		const int minX = PUCK_SIZE * 3;

		if ( pos.x < minX )
		{
			pos.x = minX;
		}
		else if ( pos.x > maxX )
		{
			pos.x = maxX;
		}
		
		cam.SetCurrPredictPos( pos );

		posY = ROBOT_DEFENSE_POSITION_DEFAULT;
		/*posX = pos.x;
		hBot.SetPosStraight( posX, posY );
*/
		m_AttackTime = 0;
	} // case 1
	break;

	case 2: // Defense+attack
	{
		if ( cam.GetPredictTimeAttack() < MIN_PREDICT_TIME ) // If time is less than 150ms we start the attack
		{
			posY = ROBOT_DEFENSE_ATTACK_POSITION_DEFAULT + PUCK_SIZE * 4; // we need some override
			posX = cam.GetPredictXAttack();

			// We supose that we start at defense position
			//com_pos_x = ROBOT_CENTER_X + (((long)(predict_x_attack - ROBOT_CENTER_X) * (com_pos_y - defense_position)) / (attack_position - defense_position));          
		}
		else      // Defense position
		{
			posY = ROBOT_DEFENSE_POSITION_DEFAULT;
			posX = cam.GetCurrPredictPos().x;  // predict_x_attack;

			m_AttackTime = 0;
		}
		hBot.SetPosStraight( posX, posY );
	} // case 2
	break;

	case 3: // ATTACK MODE
	{
		cv::Point attackPredictPos;

		if ( m_AttackTime == 0 )
		{
			attackPredictPos = cam.PredictPuckPos( 500 );

			if ( ( attackPredictPos.x > PUCK_SIZE * 3 ) &&
				( attackPredictPos.x < TABLE_WIDTH - PUCK_SIZE * 3 ) &&
				( attackPredictPos.y > PUCK_SIZE * 4 ) &&
				( attackPredictPos.y < ROBOT_CENTER_Y - PUCK_SIZE * 5 ) )
			{
				m_AttackTime = millis() + 500;  // Prepare an attack in 500ms
				Serial.print( "AM:" );
				//Serial.print(m_AttackTime);
				//Serial.print(",");
				Serial.print( attackPredictPos.x );
				Serial.print( "," );
				Serial.println( attackPredictPos.y );
				//Serial.print(" ");

				// Go to pre-attack position
				posX = attackPredictPos.x;
				posY = attackPredictPos.y - PUCK_SIZE * 4;

				const int s = MAX_ABS_SPEED * 0.5;
				hBot.SetMaxAbsSpeed( s );

				m_AttackStatus = 1;
			}
			else
			{
				m_AttackTime = 0;  // Continue waiting for the right attack moment...
				m_AttackStatus = 0;

				// And go to defense position
				posY = ROBOT_DEFENSE_POSITION_DEFAULT;
				posX = ROBOT_CENTER_X;  //center X axis
				const int s = MAX_ABS_SPEED * 0.6667; // Return a bit more slowly...
				hBot.SetMaxAbsSpeed( s );
			}

			if ( CheckOwnGoal( hBot, cam ) )
			{
				posX = hBot.GetRobotPos().x;
				posY = hBot.GetRobotPos().y;
			}

			hBot.SetPosStraight( posX, posY );
		}
		else
		{
			if ( m_AttackStatus == 1 )
			{
				long impactTime = m_AttackTime - millis();
				if ( impactTime < 170 )  // less than 150ms to start the attack
				{
					// Attack movement
					attackPredictPos = cam.PredictPuckPos( impactTime );
					posX = attackPredictPos.x;
					posY = attackPredictPos.y + PUCK_SIZE * 2;

					Serial.print( "ATTACK:" );
					Serial.print( posX );
					Serial.print( "," );
					Serial.println( posY );

					m_AttackStatus = 2; // Attacking
				}
				else  // m_AttackStatus=1 but it´s no time to attack yet
				{
					// Go to pre-attack position
					attackPredictPos = cam.PredictPuckPos( 500 );

					posX = attackPredictPos.x;
					posY = attackPredictPos.y - PUCK_SIZE * 4;

					const int s = MAX_ABS_SPEED * 0.5;
					hBot.SetMaxAbsSpeed( s );
				}

				if ( CheckOwnGoal( hBot, cam ) )
				{
					posX = hBot.GetRobotPos().x;
					posY = hBot.GetRobotPos().y;
				}

				hBot.SetPosStraight( posX, posY );

			} // if (m_AttackStatus == 1)

			if ( m_AttackStatus == 2 )
			{
				if ( millis() > ( m_AttackTime + 80 ) ) // Attack move is done? => Reset to defense position
				{
					Serial.print( "RESET" );
					m_AttackTime = 0;
					m_RobotStatus = 0;
					m_AttackStatus = 0;
				}
			} // if ( m_AttackStatus == 2 )
		} // if (m_AttackTime == 0)
	} // case 3
	break;
	case 4: // The puck came from a bounce
	{
		// Only defense now (we could improve this in future)
		// Defense mode (only move on X axis on the defense line)
		cv::Point pos = cam.GetCurrPredictPos();
		pos.x = constrain( pos.x, PUCK_SIZE * 3, TABLE_WIDTH - PUCK_SIZE * 3 );  // we leave some space near the borders...
		cam.SetCurrPredictPos( pos );

		posY = ROBOT_DEFENSE_POSITION_DEFAULT;
		posX = pos.x;
		hBot.SetPosStraight( posX, posY );

		m_AttackTime = 0;
	}
	break;

	case 5:
	{
		// User manual control
		hBot.SetMaxAbsSpeed( m_UserSpeed );
		// Control acceleration
		hBot.SetMaxAbsAccel( m_UserAccel );
		hBot.SetPosStraight( m_UserPuckPos.x, m_UserPuckPos.y );
		//Serial.println(max_acceleration);
	}// case 5
	break;

	default:
		// Default : go to defense position
		posY = ROBOT_DEFENSE_POSITION_DEFAULT;
		posX = ROBOT_CENTER_X;  //center X axis

		if ( CheckOwnGoal( hBot, cam ) )
		{
			posX = hBot.GetRobotPos().x;
			posY = hBot.GetRobotPos().y;
		}

		hBot.SetPosStraight( posX, posY );

		m_AttackTime = 0;
	}// switch
} // Robot::RobotStrategy

//====================================================================================================================
bool Robot::CheckOwnGoal( const Camera& cam )
{
	const int rotPosY = hBot.GetRobotPos().y;
	const int puckPosX = cam.GetCurrPuckPos().x;
	const int puckPosY = cam.GetCurrPuckPos().y;

	if ( rotPosY > ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE  &&  // robot in front of defence line
		puckPosY < rotPosY && // puck behind robot
		puckPosX > ROBOT_CENTER_X - PUCK_SIZE * 5 &&     // puck X within table range
		puckPosX < ROBOT_CENTER_X + PUCK_SIZE * 5 )
	{
		Serial.print( "Possible Own Goal " );
		Serial.print( puckPosX );
		Serial.print( " " );
		Serial.println( puckPosY );
		return true;
	}
	else
	{
		return false;
	}
} // CheckOwnGoal
