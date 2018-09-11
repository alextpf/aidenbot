//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#include "Robot.h"
#include "Configuration.h"

extern int freeRam ();
extern int myAbs(int param);
extern Point2I Point2Abs(Point2I param);
extern int sign(int val);

//====================================================================================================================
void Robot::NewDataStrategy( Camera& cam )
{
  m_RobotStatus = 0; // Going to initial position (defense)
  
  if ( cam.GetPredictStatus() == 1 ) // Puck comming?
  {
    if ( cam.GetPredictBounce() == 0 ) // Direct impact?
    {
      PuckPos predictPos = cam.GetCurrPredictPos();
      if ( ( predictPos.m_X > ROBOT_MIN_X + PUCK_SIZE * 2 ) && 
           ( predictPos.m_X < ROBOT_MAX_X - PUCK_SIZE * 2 ) )
      {
        // Predicted position X is within table range        
        if ( cam.GetPuckAvgSpeed().m_Y > MIN_PUCK_Y_SPEED1 )
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
        if ( cam.GetPredictTime() < PREDICT_TIME_THRESHOLD)
        {
          // predicted hit time is small, i.e. puck approcahing soon
          
          m_RobotStatus = 1; //1  // Defense
        }
        else
        {
          // predicted hit time is large, i.e. no risk          
          m_RobotStatus = 0;
        }
      }
    }
    else // Puck come from a bounce?
    {
      if ( cam.GetPuckAvgSpeed().m_Y  > MIN_PUCK_Y_SPEED2 ) // Puck is moving fast?
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
  if ( cam.GetPredictStatus() == 2)
  {
    if ( cam.GetPredictTime() < PREDICT_TIME_THRESHOLD )
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
       cam.GetCurrPuckPos().m_Y < ROBOT_CENTER_Y - 20 && 
       myAbs( cam.GetCurrPuckSpeed().m_Y ) < MIN_ABS_Y_SPEED && 
       myAbs( cam.GetCurrPuckSpeed().m_X ) < MIN_ABS_X_SPEED )
  {
    m_RobotStatus = 3; // attack
  }
} // Robot::NewDataStrategy

//====================================================================================================================
void Robot::RobotStrategy( HBot& hBot )
{
  long attackTime;
  hBot.SetMaxSpeed( MAX_SPEED ); // default to max robot speed and accel
  
  max_acceleration = user_max_accel;
  
  int posX, posY;
  switch (m_RobotStatus) 
  {
    case 0: // Go to defense position
    {      
      posY = ROBOT_DEFENSE_POSITION_DEFAULT;
      posX = ROBOT_CENTER_X;  //center X axis
      const int s = MAX_SPEED * 0.6667 // Return a bit more slowly...
      hBot.SetMaxSpeed( s );
      
      if ( CheckOwnGoal() == false )
      {
        hBot.SetPosStraight( posX, posY );
      }
      else
      {
        hBot.SetPosStraight( hBot.GetRobotPos().m_X, hBot.GetRobotPos().m_Y ); // The robot stays on it´s position
      }
      
      attack_time = 0;
    }
      break;
      
    case 1: // Defense mode (only move on X axis on the defense line)
    {      
      predict_x = constrain(predict_x, (PUCK_SIZE * 3), TABLE_WIDTH - (PUCK_SIZE * 3));  // we leave some space near the borders...
      com_pos_y = defense_position;
      com_pos_x = predict_x;
      setPosition_straight(com_pos_x, com_pos_y);
      attack_time = 0;
    }
      break;
      
    case 2: // Defense+attack
      {
        if ( predict_time_attack < MIN_PREDICT_TIME ) // If time is less than 150ms we start the attack
        {
          com_pos_y = attack_position + PUCK_SIZE * 4; // we need some override
          com_pos_x = predict_x_attack;
          // We supose that we start at defense position
          //com_pos_x = ROBOT_CENTER_X + (((long)(predict_x_attack - ROBOT_CENTER_X) * (com_pos_y - defense_position)) / (attack_position - defense_position));
          setPosition_straight(com_pos_x, com_pos_y);
        }
        else      // Defense position
        {
          com_pos_y = defense_position;
          com_pos_x = predict_x;  // predict_x_attack;
          setPosition_straight(com_pos_x, com_pos_y);
          attack_time = 0;
        }
      }
      break;
 
    case 3: // ATTACK MODE
    {
      if (attack_time == 0)
      {
        attack_predict_x = predictPuckXPosition(500);
        attack_predict_y = predictPuckYPosition(500);
        
        if ( ( attack_predict_x > PUCK_SIZE * 3 ) && 
             ( attack_predict_x < TABLE_WIDTH - PUCK_SIZE * 3 ) && 
             ( attack_predict_y > PUCK_SIZE * 4 ) && 
             ( attack_predict_y < ROBOT_CENTER_Y - PUCK_SIZE * 5 ) )
        {
          attack_time = millis() + 500;  // Prepare an attack in 500ms
          attack_pos_x = attack_predict_x;  // predict_x
          attack_pos_y = attack_predict_y;  // predict_y
          Serial.print("AM:");
          //Serial.print(attack_time);
          //Serial.print(",");
          Serial.print(attack_pos_x);
          Serial.print(",");
          Serial.println(attack_pos_y);
          //Serial.print(" ");
          // Go to pre-attack position
          com_pos_x = attack_pos_x;
          com_pos_y = attack_pos_y - PUCK_SIZE * 4;
          max_speed = user_max_speed / 2;
          if (checkOwnGoal() == false)
            setPosition_straight(com_pos_x, com_pos_y);
          else
            setPosition_straight(real_position_x, real_position_y); // The robot stays on it´s position
          attack_status = 1;
        }
        else
        {
          attack_time = 0;  // Continue waiting for the right attack moment...
          attack_status = 0;
          // And go to defense position
          com_pos_y = defense_position;
          com_pos_x = ROBOT_CENTER_X;  //center X axis
          max_speed = (user_max_speed / 3) * 2;
          if (checkOwnGoal() == false)
            setPosition_straight(com_pos_x, com_pos_y);
          else
            setPosition_straight(real_position_x, real_position_y); // The robot stays on it´s position
        }
      }
      else
      {
        if (attack_status == 1)
        {
          long impact_time = attack_time - millis();
          if ( impact_time < 170 )  // less than 150ms to start the attack
          {
            // Attack movement
            com_pos_x = predictPuckXPosition(impact_time);
            com_pos_y = predictPuckYPosition(impact_time);
            setPosition_straight(com_pos_x, (com_pos_y + PUCK_SIZE * 2));

            Serial.print("ATTACK:");
            Serial.print(com_pos_x);
            Serial.print(",");
            Serial.println(com_pos_y);

            attack_status = 2; // Attacking
          }
          else  // attack_status=1 but it´s no time to attack yet
          {
            // Go to pre-attack position
            com_pos_x = attack_pos_x;
            com_pos_y = attack_pos_y - PUCK_SIZE * 4;
            max_speed = user_max_speed / 2;
            if (checkOwnGoal() == false)
              setPosition_straight(com_pos_x, com_pos_y);
            else
              setPosition_straight(real_position_x, real_position_y); // The robot stays on it´s position
          }
        } // if (attack_status == 1)
        
        if ( attack_status == 2 )
        {
          if (millis() > (attack_time + 80)) // Attack move is done? => Reset to defense position
          {
            Serial.print("RESET");
            attack_time = 0;
//            robot_status = 0; // this line is probably of no use
            attack_status = 0;
          }
        } // if ( attack_status == 2 )
      } // if (attack_time == 0)
    }
      break;
    case 4: // The puck came from a bounce
    {
      // Only defense now (we could improve this in future)
      // Defense mode (only move on X axis on the defense line)
      predict_x = constrain(predict_x, (PUCK_SIZE * 3), TABLE_WIDTH - (PUCK_SIZE * 3));
      com_pos_y = defense_position;
      com_pos_x = predict_x;
      setPosition_straight(com_pos_x, com_pos_y);
      attack_time = 0;
    }
      break;

    case 5:
    {
      // User manual control
      max_speed = user_target_speed;
      // Control acceleration
      max_acceleration = user_target_accel;
      setPosition_straight(user_target_x, user_target_y);
      //Serial.println(max_acceleration);
    }
      break;

    default:
      // Default : go to defense position
      com_pos_y = defense_position;
      com_pos_x = ROBOT_CENTER_X; // center
      if (checkOwnGoal() == false)
        setPosition_straight(com_pos_x, com_pos_y);
      else
        setPosition_straight(real_position_x, real_position_y); // The robot stays on it´s position
      attack_time = 0;
  }// switch
} // Robot::RobotStrategy

//====================================================================================================================
bool Robot::CheckOwnGoal( const HBot& hBot, const Camera& cam )
{
  const int rotPosY = hBot.GetRobotPos().m_Y;
  const int puckPosX = cam.GetCurrPuckPos().m_X;
  const int puckPosY = cam.GetCurrPuckPos().m_Y;
  
  if ( rotPosY > ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE  &&  // robot in front of defence line
       puckPosY < rotPosY && // puck behind robot
       puckPosX > ROBOT_CENTER_X - PUCK_SIZE * 5  &&     // puck X within table range
       puckPosX < ROBOT_CENTER_X + PUCK_SIZE * 5 )
  {
    Serial.print("Possible Own Goal ");
    Serial.print(puckPosX);
    Serial.print(" ");
    Serial.println(puckPosY);
    return true;
  }
  else
  {
    return false;
  }
} // CheckOwnGoal
