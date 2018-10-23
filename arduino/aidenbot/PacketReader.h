//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////
#ifndef PACKET_READER_H
#define PACKET_READER_H

#include "Arduino.h"
#include "Point2D.h"

typedef Point2D<int> Point2I;   // 16 bit
typedef Point2D<long> Point2L;  // 32 bit
typedef Point2I RobotPos;       // alias

class PacketReader
{
public:
    PacketReader();

    void      ReadPacket();
    uint16_t  ExtractParamInt(uint8_t pos);
    
    RobotPos  GetDesiredBotPos()
    {
      return m_DesiredBotPos;
    }
    
    RobotPos  GetDetectedBotPos()
    {
      return m_DetectedBotPos;
    }
    
    int       GetDesiredMotorSpeed()
    {
      return m_DesiredMotorSpeed;
    }
    
private:

    char      m_SBuffer[12];
    bool      m_MsgFullyRead; // true: ready; false: not ready
    int       m_ReadCounter;
    RobotPos  m_DesiredBotPos;
    RobotPos  m_DetectedBotPos;
    int       m_DesiredMotorSpeed;
}; // PacketReader
#endif
