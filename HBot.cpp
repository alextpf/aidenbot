#include "HBot.h"
#include "Configuration.h"

//=========================================================
void HBot::ComputePos()
{
  m_PosX = ( m_M1.GetStep() + m_M2.GetStep() ) * 0.5 / X_AXIS_STEPS_PER_UNIT;
  m_PosY = ( m_M1.GetStep() - m_M2.GetStep() ) * 0.5 / Y_AXIS_STEPS_PER_UNIT;
}
//=========================================================

void HBot::Update() // aka positionControl()
{
  ComputePos(); // update m_PosX/Y

  // record time
  uint32_t currTime = micros();
  int16_t dt = currTime - m_Time;
  dt = constrain( currTime - m_Time, 0, 2000 );
  
  m_Time = currTime; // update time

  // update motor acceleration
  m_M1.UpdateAccel();
  m_M2.UpdateAccel();
  
  m_M1.UpdateSpeed();
  m_M2.UpdateSpeed();
  
}
