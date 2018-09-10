//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#ifndef ROBOT_H
#define ROBOT_H

#include "Camera.h"

class Robot
{
public:	
  // @brief: given a predicted status (coming toward us, no risk ... etc), 
  // determine the robot status (wheter to attack, defense...etc ).
  // @note: Each time a new data packet from camera is reveived this function is called
  void NewDataStrategy( Camera& cam );
  
private:
  // 0: Init
  // 1: Defense
  // 2: Defense+Atack
  // 3: Atack
  int8_t m_RobotStatus;  
}; // Robot

#endif
