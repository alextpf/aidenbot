#ifndef CAMERA_H
#define CAMERA_H

#include "Arduino.h"

class Camera
{
public:

  struct PuckPos
  {
    PuckPos();
    ~PuckPos();
    
    PuckPos(int8_t x, int8_t y)
    : m_PosX(x),
      m_PosY(y)
    {}
    
    int8_t m_PosX; // puck position X captured by camera, in mm
    int8_t m_PosY; // puck position Y captured by camera, in mm  

    // operators
    PuckPos operator + (const PuckPos & rhs )
    {
      return PuckPos(m_PosX + rhs.m_PosX, m_PosY + rhs.m_PosY );
    }

    PuckPos operator - (const PuckPos & rhs )
    {
      return PuckPos(m_PosX - rhs.m_PosX, m_PosY - rhs.m_PosY );
    }
  };
  
  typedef PuckPos Vector; // alias
  
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
    m_PrevPuckPos.m_PosX = pos.m_PosX;
    m_PrevPuckPos.m_PosY = pos.m_PosY;
  } // SetPrevPuckPos
  
  PuckPos GetPrevPuckPos() const
  {
    return m_PrevPuckPos;
  } // GetPrevPuckPos

  void CamProcess();
  
private:
  // position
  PuckPos m_CurrPuckPos; // current pos
  PuckPos m_PrevPuckPos; // previous pos

  // speed
  
};// Camera

#endif
