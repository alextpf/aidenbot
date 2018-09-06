//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#ifndef HBOT_H
#define HBOT_H

#include "Motor.h"
#include "Arduino.h"

class HBot
{
public:
  enum MOTOR {M1, M2};
  
  HBot(); // ctor
  ~HBot(); // dtor
  
  void SetPosInternal( int16_t x, int16_t y ); // in mm
  void SetPosStraight( int16_t x, int16_t y ); // in mm
  void UpdatePosStraight();
	void Update(); // aka positionControl()
private:
  void ComputePos(); // in mm
  
  int16_t m_PosX; // X-pos in mm. corresponds to real_position_x
  int16_t m_PosY; // Y-pos in mm. corresponds to real_position_y
	Motor m_M1;
	Motor m_M2;
	uint32_t m_Time; // time stamp, in micro sec.
};
#endif
