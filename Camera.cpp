#include "Camera.h"
#include "Configuration.h"

extern int freeRam ();
extern int16_t myAbs(int16_t param);
extern int sign(int val);

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
      
      m_CurrPredictPos.m_Y = ROBOT_DEFENSE_ATTACK_POSITION_DEFAULT + PUCK_SIZE;
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
      if (predict_bounce_status == 1)  // This is the first direct impact trajectory after a bounce
      {
        // We dont predict nothing new...
        predict_bounce_status = 0;
      }
      else
      {
        // average of the results (some noise filtering)
        if (predict_x_old > 0)
        {
          predict_x = (predict_x_old + predict_x) >> 1;
        }
        predict_x_old = predict_x;

        predict_time = ((defense_position + PUCK_SIZE) - puckCoordY) * 100L / puckSpeedY; // in ms
        predict_time_attack = ((attack_position + PUCK_SIZE) - puckCoordY) * 100L / puckSpeedY; // in ms
        predict_time -= VISION_SYSTEM_LAG;
        predict_time_attack -= VISION_SYSTEM_LAG;
      }
    }
  }
  else // // Puck is moving slowly or to the other side
  {
    
  }//if ( m_AverageSpeed.m_Y < -50 )
} // CamProcess
