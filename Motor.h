//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor
{
public:
  Motor();
  ~Motor();
  
  void SetStep( int step )
  {
  	m_Step = step;
  }

  int GetStep() const
  {
  	return m_Step;
  }

  void SetGoalStep( int step )
  {
  	m_GoalStep = step;
  }

  int GetGoalStep() const
  {
  	return m_GoalStep;
  }

  void SetSpeed( int speed )
  {
    m_Speed = speed;
  }

  int GetSpeed() const
  {
    return m_Speed;
  }

  void SetGoalSpeed( int speed )
  {
    m_GoalSpeed = speed;
  }

  int GetGoalSpeed() const
  {
    return m_GoalSpeed;
  }
  
  void SetAccel( int accel )
  {
    m_Accel = accel;
  }

  int GetAccel() const
  {
    return m_Accel;
  }
  
  void SetMaxAccel( int accel )
  {
    m_MaxAccel = accel;
  }

  int GetMaxAccel()
  {
    return m_MaxAccel;
  }

  int8_t GetDir() const
  {
    return m_Dir;
  }
  
  void SetDir( int8_t dir )
  {
    m_Dir = dir;
  }

  long GetPeriod() const
  {
    return m_Period;
  }
  
  void SetMaxSpeed( int s )
  {
     m_MaxSpeed = s;
  }
  
  int GetMaxSpeed()
  {
    return m_MaxSpeed;
  }
  
  void UpdateAccel();
  void UpdateSpeed( int dt );
  
private:  
  void SetSpeedInternal( int dt, int goalSpeed );

  //////////////
  // Position
  //////////////
	int m_Step;       // in motor steps. corresponds to position_M1/2
  int m_GoalStep;   // corresponds to target_position_M1/2

  //////////////
  // Direction
  //////////////
  int8_t m_Dir;         // dir = 1 movepositive, dir = -1 move negative, dir = 0 , no move

  ///////////////////
  // Speed & Accel
  ///////////////////
	int m_Accel;      // acceleration. corresponds to acceleration_M1/2
  int m_Speed;      // corresponds to speed_M1/2
  int m_GoalSpeed;  // corresponds to target_speed_M1/2
  int m_MaxSpeed;   // corresponds to max_speed

  int m_MaxAccel;   // max acceleration
  long m_Period;    // for setting timer use. corresponds to timer_period. arduino "long" is 32 bit
  
};

#endif
