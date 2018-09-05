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
  
  void SetPos( int16_t x, int16_t y, bool isGoal ); // in mm
	void Update(); // aka positionControl()
private:
  void ComputePos(); // in mm
  void UpdateTimer( MOTOR m );
  
  int16_t m_PosX; // X-pos in mm. corresponds to real_position_x
  int16_t m_PosY; // Y-pos in mm. corresponds to real_position_y
	Motor m_M1;
	Motor m_M2;
	uint32_t m_Time; // time stamp, in micro sec.
};
#endif
