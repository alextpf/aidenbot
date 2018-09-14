//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#ifndef ROBOT_H
#define ROBOT_H

#include "Camera.h"
#include "HBot.h"

class Robot
{
public:	
  Robot();
  ~Robot();
  //====================================================================================================================
  // @brief: given a predicted status (coming toward us, no risk ... etc), 
  // determine the robot status (wheter to attack, defense...etc ).
  // @note: Each time a new data packet from camera is reveived this function is called
  void NewDataStrategy( Camera& cam );
    
  //====================================================================================================================
  // Robot Moves depends directly on robot status
  //
  // robot status:
  //   0: Go to defense position
  //   1: Defense mode (only move on X axis on the defense line)
  //   2: Defense + attach mode
  //   3: Attack mode
  //   4: ?? REMOVE ??
  //   5: Manual mode => User send direct commands to robot
  void RobotStrategy( HBot& hBot, Camera& cam );

  
  //====================================================================================================================
  // This function returns true if the puck is behind the robot and 
  // there are posibilities of an auto goal when the robots moves back
  bool CheckOwnGoal( const HBot& hBot, const Camera& cam );
  
private:
  // 0: Init
  // 1: Defense
  // 2: Defense+Atack
  // 3: Atack
  int8_t m_RobotStatus;  
  long m_AttackTime;
  uint8_t m_AttackStatus;

  int m_UserSpeed;
  int m_UserAccel;
  PuckPos m_UserPuckPos;
}; // Robot

#endif
