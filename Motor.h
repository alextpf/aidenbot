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
  enum MOTOR_NUM {M1,M2};
  
  Motor();
  ~Motor();
  
  void SetCurrStep( int step )
  {
  	m_CurrStep = step;
  }

  int GetCurrStep() const
  {
  	return m_CurrStep;
  }

  void SetGoalStep( int step )
  {
  	m_GoalStep = step;
  }

  int GetGoalStep() const
  {
  	return m_GoalStep;
  }

  void SetCurrSpeed( int speed )
  {
    m_CurrSpeed = speed;
  }

  int GetCurrSpeed() const
  {
    return m_CurrSpeed;
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
  
  void SetMaxAbsAccel( int accel )
  {
    m_MaxAbsAccel = accel;
  }

  int GetMaxAbsAccel()
  {
    return m_MaxAbsAccel;
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
  
  void SetMaxAbsSpeed( int s )
  {
     m_MaxAbsSpeed = s;
  }
  
  int GetMaxAbsSpeed()
  {
    return m_MaxAbsSpeed;
  }
  
  void UpdateAccel();
  void UpdateSpeed( int dt, MOTOR_NUM m );
  
private:  
  void SetCurrSpeedInternal( int dt, int goalSpeed, MOTOR_NUM m );

  //////////////
  // Position
  //////////////
	int m_CurrStep;       // in motor steps. corresponds to position_M1/2
  int m_GoalStep;       // corresponds to target_position_M1/2

  //////////////
  // Direction
  //////////////
  int8_t m_Dir;         // dir = 1 movepositive, dir = -1 move negative, dir = 0 , no move

  ///////////////////
  // Speed & Accel
  ///////////////////
	int m_CurrSpeed;      // signed speed. corresponds to speed_M1/2
  int m_GoalSpeed;      // signed speed. corresponds to target_speed_M1/2
  int m_MaxAbsSpeed;       // unsighed speed. corresponds to max_speed

  int m_Accel;          // signed acceleration. corresponds to acceleration_M1/2
  int m_MaxAbsAccel;       // unsighed max acceleration
  
  long m_Period;        // for setting timer use. corresponds to timer_period. arduino "long" is 32 bit  
};

#endif
