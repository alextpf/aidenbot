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
  // @brief use algorithm to calculate goal speed and set m_GoalSpeed for M1 & M2
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

  const RobotPos& GetRobotPos() const
  {
    return m_Pos;
  }

  void SetMaxSpeed( int s )
  {
     m_M1.SetMaxSpeed(s);
     m_M2.SetMaxSpeed(s);
  }

  int GetMaxSpeed()
  {
    return m_M1.GetMaxSpeed(); // assuming max speed for 2 motors are the same
  }
  
  void SetMaxAccel( int accel )
  {
    m_M1.SetMaxAccel( accel );
    m_M2.SetMaxAccel( accel );
  }  
  
  int GetMaxAccel()
  {
    return m_M1.GetMaxAccel(); // assuming max accel for 2 motors are the same
  }

  void SetTime( long t )
  {
    m_Time = t;
  }

  
  // utility function
  static RobotPos MotorStepToHBotPos(int m1Step, int m2Step); // in mm
  static void HBotPosToMotorStep(const RobotPos& pos, int& m1Step, int& m2Step); // in mm
private:  

  RobotPos m_Pos;
	Motor m_M1;
	Motor m_M2;
	long m_Time; // time stamp, in micro sec.  
};
#endif
