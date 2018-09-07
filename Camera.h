#ifndef CAMERA_H
#define CAMERA_H

#include "Arduino.h"
#include "Point2D.h"

typedef Point2D<int> Point2I;   // 16 bit
typedef Point2D<long> Point2L;  // 32 bit

typedef Point2I PuckPos;        // alias
typedef Point2I Vector;         // alias
typedef Point2L PuckSpeed;      // alias

class Camera
{
public:
  
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
  
private:
  //////////////
  // position
  //////////////
  PuckPos m_CurrPuckPos; // current pos. mm
  PuckPos m_PrevPuckPos; // previous pos. mm
  PuckPos m_CurrPredictPos;
  PuckPos m_PrevPredictPos;
  int m_PredictXAttack; // predicted X coordinate for attack

  //////////////
  // speed
  //////////////
  PuckSpeed m_CurrPuckSpeed; // current speed. dm/ms
  PuckSpeed m_PrevPuckSpeed; // previous speed. dm/ms
  PuckSpeed m_AverageSpeed;

  
  // 0 : No risk, 
  // 1 : Puck is moving to our field directly, 
  // 2 : Puck is moving to our field with a bounce
  // 3 : ?
  // -1 : error: noise
  int8_t m_PredictStatus;
};// Camera

#endif
