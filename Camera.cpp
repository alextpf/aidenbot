#include "Camera.h"
#include "Configuration.h"

extern int freeRam ();
extern int myAbs(int param);
extern Point2I Point2Abs(Point2I param);
extern int sign(int val);

//=========================================================
Camera::Camera()
: m_PredictXAttack( 0 )
, m_PredictStatus( 0 )
, m_PredictBounce( 0 )
, m_PredictBounceStatus( 0 )
, m_PredictTime( 0 )
, m_PredictTimeAttack( 0 )
{}

//=========================================================
Camera::~Camera()
{}

//=========================================================
void Camera::CamProcess( int dt /*ms*/ )
{  
  // Speed calculation on each axis  
  Vector vec = m_CurrPuckPos - m_PrevPuckPos;
  Point2L tmpVec( vec.m_X, vec.m_Y ); // tmp var, conversion from int (16 bit) to long (32 bit)
  
  m_PrevPuckSpeed = m_CurrPuckSpeed; // update old speed
  m_CurrPuckSpeed = tmpVec * 100 / dt; // speed in dm/ms (we use this units to not overflow the variable)
  
  // Noise detection, if there are a big speeds this should be noise
  if ( m_CurrPuckSpeed.m_X < -1000 || 
       m_CurrPuckSpeed.m_X > 1000 || 
       m_CurrPuckSpeed.m_Y < -1000 || 
       m_CurrPuckSpeed.m_Y > 1000 )
  {
    Serial.println("NOISE");
    m_PredictStatus = -1;
    m_PrevPredictPos.m_X = -1;
    return;
  }
  
  if ( m_PredictStatus == -1 )  // Noise on last reading?
  {
    m_AverageSpeed = m_CurrPuckSpeed;
  }
  else
  {
    // if there are low accelerations (similar speeds on readings) we apply an average filtering with the previous value...
    m_AverageSpeed.m_X = myAbs( m_CurrPuckSpeed.m_X - m_PrevPuckSpeed.m_X ) < 50 ?
      ( m_CurrPuckSpeed.m_X + m_PrevPuckSpeed.m_X ) >> 1 : m_CurrPuckSpeed.m_X;
      
    m_AverageSpeed.m_Y = myAbs( m_CurrPuckSpeed.m_Y - m_PrevPuckSpeed.m_Y ) < 50 ?
      ( m_CurrPuckSpeed.m_Y + m_PrevPuckSpeed.m_Y ) >> 1 : m_CurrPuckSpeed.m_Y;
  }

  m_PredictXAttack = -1;
  
  // It´s time to predict...
  // Based on actual position and move vector we need to know the future...
  // Posible impact? speed Y is negative when the puck is moving to the robot
  if ( m_AverageSpeed.m_Y < -50 )  //-25
  {
    m_PredictStatus = 1;
    
    // Puck is comming...
    // We need to predict the puck position when it reaches our goal Y position = defense_position
    // slope formula: m = (y2-y1)/(x2-x1)
    float slope = vec.m_X == 0 ?   // To avoid division by 0
      9999999 : (float)vec.m_Y / (float)vec.m_X;
    
    // Prediction of the new x position at defense position: x2 = (y2-y1)/m + x1
    m_CurrPredictPos.m_Y = ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE;
    m_CurrPredictPos.m_X = ( m_CurrPredictPos.m_Y - m_CurrPuckPos.m_Y ) / slope + m_CurrPuckPos.m_X;
    
    // Prediction of the new x position at attack position
    m_PredictXAttack = ( ROBOT_DEFENSE_ATTACK_POSITION_DEFAULT + PUCK_SIZE - m_CurrPuckPos.m_Y) / slope + m_CurrPuckPos.m_X;

    // puck has a bounce with side wall?
    if ( m_CurrPredictPos.m_X < PUCK_SIZE || 
         m_CurrPredictPos.m_X > TABLE_WIDTH - PUCK_SIZE )
    {
      m_PredictStatus = 2;
      m_PredictBounce = 1;
      m_PredictBounceStatus = 1;
      
      // We start a new prediction

      PuckPos bouncePos;
      
      // Wich side?
      bouncePos.m_X = m_CurrPredictPos.m_X < PUCK_SIZE ?
          PUCK_SIZE /*Left side*/: TABLE_WIDTH - PUCK_SIZE /*Right side*/;
          
      bouncePos.m_Y = ( bouncePos.m_X - m_CurrPuckPos.m_X ) * slope + m_CurrPuckPos.m_Y;
      
      m_PredictTime = ( bouncePos.m_Y - m_CurrPuckPos.m_Y ) * 100L / m_CurrPuckSpeed.m_Y; // time until bouce
      
      // bounce prediction => slope change  with the bounce, we only need to change the sign, easy!!
      slope = -slope;
      
      m_CurrPredictPos.m_Y = ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE;
      m_CurrPredictPos.m_X = ( m_CurrPredictPos.m_Y - bouncePos.m_Y ) / slope + bouncePos.m_X;
      
      if ( m_CurrPredictPos.m_X < PUCK_SIZE || 
           m_CurrPredictPos.m_X > TABLE_WIDTH - PUCK_SIZE ) // New bounce with side wall?
      {
        // We do nothing then... with two bounces there are small risk of goal...
        m_PrevPredictPos.m_X = -1;
        m_PredictStatus = 0; // no risk
      }
      else
      {
        // only one side bounce...
        // If the puckSpeedY has changed a lot this mean that the puck has touch one side
        if ( myAbs( m_CurrPuckSpeed.m_Y - m_PrevPuckSpeed.m_Y ) > 50 )
        {
          // We dont make a new prediction...
          m_PrevPredictPos.m_X = -1;
        }
        else
        {
          // average of the results (some noise filtering)
          if ( m_PrevPredictPos.m_X != -1 )
          {
            m_CurrPredictPos.m_X = ( m_PrevPredictPos.m_X + m_CurrPredictPos.m_X ) >> 1;
          }
          
          m_PrevPredictPos.m_X = m_CurrPredictPos.m_X;
          
          // We introduce a factor (120 instead of 100) to model the bounce (20% loss in speed)(to improcve...)
          m_PredictTime += ( m_CurrPredictPos.m_Y - m_CurrPuckPos.m_Y ) * 120L / m_CurrPuckSpeed.m_Y; // in ms
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
        if ( m_PrevPredictPos.m_X > 0 )
        {
          m_CurrPredictPos.m_X = ( m_PrevPredictPos.m_X + m_CurrPredictPos.m_X ) >> 1;
        }
        m_PrevPredictPos.m_X = m_CurrPredictPos.m_X;

        m_PredictTime = ( ROBOT_DEFENSE_POSITION_DEFAULT + PUCK_SIZE - m_CurrPuckPos.m_Y ) * 100L / m_CurrPuckSpeed.m_Y - VISION_SYSTEM_LAG; // in ms
        m_PredictTimeAttack = ( ROBOT_DEFENSE_ATTACK_POSITION_DEFAULT + PUCK_SIZE - m_CurrPuckPos.m_Y ) * 100L / m_CurrPuckSpeed.m_Y - VISION_SYSTEM_LAG; // in ms        
      }
    }
  }
  else // // Puck is moving slowly or to the other side
  {
    m_PrevPredictPos.m_X = -1;
    m_PredictStatus = 0;
    m_PredictBounce = 0;
    m_PredictBounceStatus = 0;
  }//if ( m_AverageSpeed.m_Y < -50 )
} // CamProcess

//=========================================================
PuckPos Camera::PredictPuckPos( int predictTime )
{
  predictTime += VISION_SYSTEM_LAG;
  Point2L tmpPos( m_AverageSpeed * m_PredictTime / 100L );
  PuckPos tmp(tmpPos.m_X, tmpPos.m_Y);
  return m_CurrPuckPos + tmp;
} // PredictPuckYPos

//=========================================================
void Camera::MissingStepsDetection( HBot& hBot )
{
  // if we don´t have a valid robot detection from camera => exit
  if ( m_RobotPos == RobotPos( 0, 0 ) )
  {
    return;
  }
  
  int robotCoordSamples = 0;
  RobotPos robotPosAvg(0,0);
  RobotPos robotMissingStepsError(0,0);
  
  // If we are stopped and robot corrdinates are OK...
  if ( myAbs( hBot.GetM1().GetSpeed() ) < 400 && 
       myAbs( hBot.GetM2().GetSpeed() ) < 400 && 
       m_RobotPos.m_X < TABLE_WIDTH && 
       m_RobotPos.m_Y < TABLE_LENGTH * 0.5 )
  {
    // Are we near center and near defense position?
    if ( hBot.GetRobotPos().m_X  > ROBOT_CENTER_X - 50 && 
         hBot.GetRobotPos().m_X  < ROBOT_CENTER_X + 50 && 
         hBot.GetRobotPos().m_Y  >= ROBOT_MIN_Y && 
         hBot.GetRobotPos().m_Y  < ROBOT_DEFENSE_POSITION_DEFAULT + 60 )
    {
      robotCoordSamples++;
      robotPosAvg += hBot.GetRobotPos();
      
      // When we collect 10 samples we make the correction
      if (robotCoordSamples == 10)
      {        
        robotPosAvg /= robotCoordSamples;
        robotMissingStepsError = Point2Abs( robotPosAvg - hBot.GetRobotPos() );  // in mm
        
        // robot_position_y_mm += ROBOT_POSITION_CAMERA_CORRECTION_Y;   // correction because camera point of view and robot mark
        
        if ( robotMissingStepsError.m_X > MISSING_STEPS_MAX_ERROR_X ||
             robotMissingStepsError.m_Y > MISSING_STEPS_MAX_ERROR_Y )
        {
          // Missing steps detected We need to correct this...
#ifdef CORRECT_MISSING_STEPS
          int m1s,m2s;
          HBot::HBotPosToMotorStep( robotPosAvg, m1s, m2s );
          hBot.GetM1().SetStep( m1s );
          hBot.GetM2().SetStep( m2s );
          
          Serial.print("MSX:");
          Serial.println(robotMissingStepsError.m_X);
          Serial.print("MSY:");
          Serial.println(robotMissingStepsError.m_Y);
          //max_acceleration = user_max_accel / 2;          
#endif
        }
        robotCoordSamples = 0; 
        robotPosAvg.m_X = 0;
        robotPosAvg.m_Y = 0;
      }
    }
    else
    {
      robotCoordSamples = 0;
      robotPosAvg.m_X = 0;
      robotPosAvg.m_Y = 0;
      robotMissingStepsError.m_X = 0;
      robotMissingStepsError.m_Y = 0;
    }
  }
  else
  {
    robotCoordSamples = 0;
    robotPosAvg.m_X = 0;
    robotPosAvg.m_Y = 0;
    robotMissingStepsError.m_X = 0;
    robotMissingStepsError.m_Y = 0;
  }
}
 // MissingStepsDetection
