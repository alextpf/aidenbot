#include "PacketReader.h"

void PacketReader::ReadPacket()
{
  char SBuffer[12];
  
  if (Serial.available() > 0) 
  {
    //Serial.println("P");
    // We rotate the Buffer (we could implement a ring buffer in future)
    for (int i=11;i>0;i--)
    {
      SBuffer[i] = SBuffer[i-1];
    }

    SBuffer[0] = Serial.read();
    //Serial.print(S1Buffer[0]);
    // We look for a  message start like "AA" to sync packets
    if ((SBuffer[0] == 'A')&&(SBuffer[1] == 'A'))
    {
      if (readStatus == 0)
      {
        readStatus=1;
        readCounter=12;
      }
      else
      {
        Serial.println("S ERR");
        readStatus=1;
        readCounter=12;
      }
      return;
    }
    else if (readStatus==1)
    {
      readCounter--;   // Until we complete the packet
      if (readCounter<=0)   // packet complete!!
      {
        // Extract parameters
        cam_timestamp = extractParamInt(10);
        puckPixX = extractParamInt(8);
        puckPixY = extractParamInt(6);
        puckSize = extractParamInt(4);
        robotPixX = extractParamInt(2);
        robotPixY = extractParamInt(0);  
        readStatus = 0;
        newPacket = 1;
        //Serial.println("P");
      }
    }
  }
}
