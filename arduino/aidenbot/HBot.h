//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#ifndef HBOT_H
#define HBOT_H

#include "Motor.h"
#include "Arduino.h"
#include "Point2D.h"

typedef Point2D<int> Point2I;   // 16 bit
typedef Point2D<long> Point2L;  // 32 bit
typedef Point2I RobotPos;       // alias

class HBot
{
public:
  
  HBot(); // ctor
  ~HBot(); // dtor

  //==================================================================================================================
  // @brief set m_GoalStep for M1 & M2
  //==================================================================================================================
  void SetPosInternal( int x, int y ); // in mm

  //==================================================================================================================
  void SetPosStraight( int x, int y ); // in mm

  //==================================================================================================================
  // @brief use algorithm to calculate goal speed and set m_AbsGoalSpeed for M1 & M2
  //==================================================================================================================
  void UpdatePosStraight();
  
  //==================================================================================================================
  // @brief 
  // 1. update accel by: the faster the current speed, the higher the accel
  // 2. based on the target position, compute a "stopping position", based on which, decite whether we increase
  //    or decrease current speed. The amount of in/decrement is by accel
  // 3. based on speed, determine the update period. 
  //==================================================================================================================
	void Update(); // aka positionControl()

  //==================================================================================================================
  Motor& GetM1()
  {
    return m_M1;
  }
  
  Motor& GetM2()
  {
    return m_M2;
  }

  void SetRobotPos(const RobotPos& pos)
  {
    m_Pos = pos;
  }
  
  const RobotPos& GetRobotPos() const
  {
    return m_Pos;
  }
  
  void SetXMaxAbsSpeed( int s )
  {
     m_M1.SetMaxAbsSpeed(s);
  }
  
  void SetYMaxAbsSpeed( int s )
  {
     m_M2.SetMaxAbsSpeed(s);
  }

  int GetMaxAbsSpeed()
  {
    return m_M1.GetMaxAbsSpeed(); // assuming max speed for 2 motors are the same
  }
  
  void SetXMaxAbsAccel( int accel )
  {
    m_M1.SetMaxAbsAccel( accel );
  }  
  
  void SetYMaxAbsAccel( int accel )
  {
    m_M2.SetMaxAbsAccel( accel );
  }  
  
  int GetMaxAbsAccel()
  {
    return m_M1.GetMaxAbsAccel(); // assuming max accel for 2 motors are the same
  }

  void SetTime( long t )
  {
    m_Time = t;
  }

  unsigned long GetLoopCounter()
  {
    return m_LoopCounter; 
  }
  
  // utility function
  static RobotPos MotorStepToHBotPos(long m1Step, long m2Step); // in mm
  static void HBotPosToMotorStep(const RobotPos& pos, long& m1Step, long& m2Step); // in mm
private:  

  RobotPos m_Pos;
	Motor m_M1; // control X-axis
	Motor m_M2; // control Y-axis. For 2-motor system, this represents 1 motor; for 3-motor system, this represents 2 motor (in sync)
	uint32_t m_Time; // time stamp, in micro sec. 
  unsigned long m_LoopCounter; 
};
#endif
