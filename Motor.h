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
  
  void SetStep( int16_t step )
  {
  	m_Step = step;
  }

  int16_t GetStep() const
  {
  	return m_Step;
  }

  void SetGoalStep( int16_t step )
  {
  	m_GoalStep = step;
  }

  int16_t GetGoalStep() const
  {
  	return m_GoalStep;
  }

  void SetSpeed( int16_t speed )
  {
    m_Speed = speed;
  }

  int16_t GetSpeed() const
  {
    return m_Speed;
  }

  void SetGoalSpeed( int16_t speed )
  {
    m_GoalSpeed = speed;
  }

  int16_t GetGoalSpeed() const
  {
    return m_GoalSpeed;
  }
  
  void SetAccel( int16_t accel )
  {
    m_Accel = accel;
  }

  int16_t GetAccel() const
  {
    return m_Accel;
  }
  
  void SetMaxAccel( int16_t accel )
  {
    m_MaxAccel = accel;
  }

  int16_t GetMaxAccel()
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
  
  void UpdateAccel();
  void UpdateSpeed( int16_t dt );
  
private:  
  void SetSpeedInternal( int16_t dt, int16_t goalSpeed );

  //////////////
  // Position
  //////////////
	int16_t m_Step;       // in motor steps. corresponds to position_M1/2
  int16_t m_GoalStep;   // corresponds to target_position_M1/2

  //////////////
  // Direction
  //////////////
  int8_t m_Dir;         // dir = 1 movepositive, dir = -1 move negative, dir = 0 , no move

  ///////////////////
  // Speed & Accel
  ///////////////////
	int16_t m_Accel;      // acceleration. corresponds to acceleration_M1/2
  int16_t m_Speed;      // corresponds to speed_M1/2
  int16_t m_GoalSpeed;  // corresponds to target_speed_M1/2

  int16_t m_MaxSpeed;   // max speed
  int16_t m_MaxAccel;   // max acceleration
  long    m_Period;     // for setting timer use. corresponds to timer_period. arduino "long" is 32 bit
  
};

#endif
