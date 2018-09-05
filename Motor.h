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

  int16_t GetStep()
  {
  	return m_Step;
  }

  void SetGoalStep( int16_t step )
  {
  	m_GoalStep = step;
  }

  int16_t GetGoalStep()
  {
  	return m_GoalStep;
  }

  void SetSpeed( int16_t speed )
  {
    m_Speed = speed;
  }

  int16_t GetSpeed()
  {
    return m_Speed;
  }

  void SetGoalSpeed( int16_t speed )
  {
    m_GoalSpeed = speed;
  }

  int16_t GetGoalSpeed()
  {
    return m_GoalSpeed;
  }
  
  void SetAccel( int16_t accel )
  {
    m_Accel = accel;
  }

  int16_t GetAccel()
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
  
  void UpdateAccel();
  void UpdateSpeed();
  
private:  
	int16_t m_Step; // in motor steps. corresponds to position_M1/2
  int16_t m_GoalStep; // corresponds to target_position_M1/2

	int16_t m_Accel; // acceleration. corresponds to acceleration_M1/2
  int16_t m_Speed; // corresponds to speed_M1/2
  int16_t m_GoalSpeed; // corresponds to target_speed_M1/2

  int16_t m_MaxAccel; // max acceleration
};

#endif
