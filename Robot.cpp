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

