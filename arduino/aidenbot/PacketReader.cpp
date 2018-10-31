#include "PacketReader.h"
#include "Configuration.h"

//==========================================================================
PacketReader::PacketReader()
    : m_DesiredMotorSpeed( MAX_ABS_SPEED )
    , m_IsPacketRead( false )
{}

//==========================================================================
bool PacketReader::ReadPacket()
{
    const byte numBytes = 10;
    static bool isInSync = false; // true: sync chars read; false: not ready
    static byte idx = 0;

    //Serial.println("pHEllo");
    //return;

    char c1,c2;
    bool ret = false;
    
    if ( Serial.available() > 0 )
    {
        //debug
        Serial.println("in 1st available");

        c1 = Serial.read();
        
        while ( Serial.available() > 0 && !m_IsPacketRead )
        {
            //debug
            Serial.println("in while available");
            
            c2 = Serial.read();
            
            // We look for a  message start like "AA" to sync packets            
            if( isInSync )
            {
                //debug
                Serial.println("start count and read packet");

                bool startRead = true;

                if( c2 == 'B' )
                {
                    c1 = Serial.read();
                    if( c1 == 'B' )
                    {   
                        Serial.println("is done");
                        
                        //done                     
                        startRead = false;                        
                        isInSync = false;
                        idx = 0;
                        m_IsPacketRead = true;

                        ret = true;
                    }
                }

                if( startRead )
                {
                    Serial.println( "start reading " );
                    
                    m_Buffer[idx] = c2;
                    idx++;
                    if ( idx >= numBytes )
                    {
                        idx = numBytes - 1;

                        Serial.println( "possible error " );
                        ret = false;
                    }
                }
            }
            else if( ( c1 == 'A' ) && ( c2 == 'A' ) )
            {
                //debug
                Serial.println("in AA");
                isInSync = true;
            }

            c1 = c2;
        }

        // Extract parameters
        m_DesiredBotPos.m_X = ExtractParamInt( 0 );
        m_DesiredBotPos.m_Y = ExtractParamInt( 2 );
        m_DetectedBotPos.m_X = ExtractParamInt( 4 );
        m_DetectedBotPos.m_Y = ExtractParamInt( 6 );
        m_DesiredMotorSpeed = ExtractParamInt( 8 );    
    }

    return ret;
} // ReadPacket

//==========================================================================
uint16_t PacketReader::ExtractParamInt(uint8_t pos)
{
  union{
    byte Buff[2];
    uint16_t d;
  }
  u;

  u.Buff[0] = m_Buffer[pos];
  u.Buff[1] = m_Buffer[pos+1];
  
  return (u.d); 
} // ExtractParamInt

//DEBUG
void PacketReader::recvBytesWithStartEndMarkers() 
{
    const byte numBytes = 11;
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte startMarker = 0x3C;
    byte endMarker = 0x3E;
    byte rb;
   

    while (Serial.available() > 0 && m_IsPacketRead == false) 
    {
        rb = Serial.read();

        if (recvInProgress == true) 
        {
            if (rb != endMarker) 
            {                
                m_Buffer[ndx] = rb;
                ndx++;
                
                if (ndx >= numBytes) 
                {
                    ndx = numBytes - 1;
                }
            }
            else 
            {
                m_Buffer[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                m_IsPacketRead = true;
            }
        }
        else if (rb == startMarker) 
        {
            recvInProgress = true;
        }
    }
}

void PacketReader::showNewData() 
{
    if (m_IsPacketRead == true) 
    {
        Serial.print("This just in (HEX values)... ");
        for (byte n = 0; n < 10; n++) {
            Serial.print(m_Buffer[n], HEX);
            Serial.print(' ');
        }
        Serial.println();
        m_IsPacketRead = false;
    }
}

