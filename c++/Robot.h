//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>
#include <time.h>

#include "Camera.h"

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
  void RobotStrategy( Camera& cam );
  
  //====================================================================================================================
  // This function returns true if the puck is behind the robot and 
  // there are posibilities of an auto goal when the robots moves back
  bool CheckOwnGoal( const Camera& cam );
  
private:
  // 0: Init
  // 1: Defense
  // 2: Defense+Atack
  // 3: Atack
  unsigned int		m_RobotStatus;  
  clock_t			m_AttackTime;
  unsigned int		m_AttackStatus;

  cv::Point			m_UserPuckPos;
}; // Robot
