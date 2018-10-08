#ifndef CAMERA_H
#define CAMERA_H

#include "Arduino.h"
#include "Point2D.h"
#include "HBot.h"

typedef Point2I PuckPos;        // alias
typedef Point2I Vector;         // alias
typedef Point2L PuckSpeed;      // alias

class Camera
{
public:
  Camera();
  ~Camera();
  
  void SetCurrPuckPos( const PuckPos pos )
  {
    m_CurrPuckPos = pos;
  } // SetCurrPuckPos
  
  PuckPos GetCurrPuckPos() const
  {
    return m_CurrPuckPos;
  } // GetCurrPuckPos

  void SetPrevPuckPos( const PuckPos pos )
  {
    m_PrevPuckPos = pos;
  } // SetPrevPuckPos
  
  PuckPos GetPrevPuckPos() const
  {
    return m_PrevPuckPos;
  } // GetPrevPuckPos

  void CamProcess( int dt /*ms*/ );
  
  int8_t GetPredictBounce()
  {
    return m_PredictBounce;
  } // GetPredictBounce

  int GetPredictTime() const
  {
    return m_PredictTime;
  }
  
  int GetPredictTimeAttack() const
  {
    return m_PredictTimeAttack;
  }

  PuckPos PredictPuckPos( int predictTime );

  // Function to detect missing steps in steppers
  // When the robot is stopped in a known position (defense position) 
  // we compare the estimated position from steppers with the position
  // of the robot seen in the camera.
  void MissingStepsDetection( HBot& hBot );

  int8_t GetPredictStatus() const
  {
    return m_PredictStatus;
  }
  
  PuckPos GetCurrPredictPos() const
  {
    return m_CurrPredictPos;
  }

  void SetCurrPredictPos(const PuckPos& pos )
  {
    m_CurrPredictPos = pos;
  }
  
  PuckSpeed GetPuckAvgSpeed() const
  {
    return m_AverageSpeed;
  }

  PuckSpeed GetCurrPuckSpeed() const
  {
    return m_CurrPuckSpeed;
  }

  int GetPredictXAttack()
  {
    return m_PredictXAttack;
  }
   
private:
  /////////////////////////////
  // Puck
  /////////////////////////////
   
  //////////////
  // position
  //////////////
  PuckPos   m_CurrPuckPos;        // current pos. mm. Corresponds to puckCoordX/Y. Updated in reading from the camera
  PuckPos   m_PrevPuckPos;        // previous pos. mm. Corresponds to puckOldCoordX. Updated in reading from the camera, when new puck pos comes in
  PuckPos   m_CurrPredictPos;
  PuckPos   m_PrevPredictPos;
  int       m_PredictXAttack;     // predicted X coordinate for attack

  //////////////
  // speed
  //////////////
  PuckSpeed m_CurrPuckSpeed;      // current speed. dm/ms
  PuckSpeed m_PrevPuckSpeed;      // previous speed. dm/ms
  PuckSpeed m_AverageSpeed;
  
  // 0 : No risk, 
  // 1 : Puck is moving to our field directly, 
  // 2 : Puck is moving to our field with a bounce
  // 3 : ?
  // -1 : error: noise
  int8_t    m_PredictStatus;

  // Bounce
  int8_t    m_PredictBounce;     // number of bounce predicted
  int8_t    m_PredictBounceStatus; //

  // Time
  int       m_PredictTime;
  int       m_PredictTimeAttack;
  
  /////////////////////////////
  // Robot
  /////////////////////////////  
  RobotPos  m_RobotPos;        // robot pos captured by camera. mm
  
};// Camera

#endif
