#include "PacketReader.h"
#include "Configuration.h"

//==========================================================================
PacketReader::PacketReader()
    : m_InSync( false )
    , m_ReadCounter( 0 )
    , m_DesiredMotorSpeed( MAX_ABS_SPEED )
{}

//==========================================================================
bool PacketReader::ReadPacket()
{
    //Serial.println("pHEllo");
    //return;
    bool ret = false;
    
    if( Serial.available() > 0 )
    {
        //debug
        Serial.println("in available");
        
        //char input = Serial.read();
        
        // We rotate the Buffer (we could implement a ring buffer in future)
        for( int i = 11; i > 0; i-- )
        {
            m_SBuffer[i] = m_SBuffer[i - 1];
        }

        m_SBuffer[0] = Serial.read();
        //Serial.print(SBuffer[0]);
        // We look for a  message start like "AA" to sync packets
        if( ( m_SBuffer[0] == 'A' ) && ( m_SBuffer[1] == 'A' ) )
        {
            //debug
            Serial.println("in AA");
        
            if( m_InSync )
            {
                Serial.println( "S ERR" );
            }
            
            m_InSync = true;
            m_ReadCounter = 12;
        }
        else if( m_InSync )
        {
            Serial.println("start counter decrease");
            m_ReadCounter--;   // Until we complete the packet
            if( m_ReadCounter <= 0 )   // packet complete!!
            {
                // Extract parameters
                m_DesiredBotPos.m_X = ExtractParamInt( 8 );
                m_DesiredBotPos.m_Y = ExtractParamInt( 6 );
                m_DetectedBotPos.m_X = ExtractParamInt( 4 );
                m_DetectedBotPos.m_Y = ExtractParamInt( 2 );
                m_DesiredMotorSpeed = ExtractParamInt( 0 );
                m_InSync = false;

                ret = true;
            }
        }
    }
    
    return ret;
} // ReadPacket

//==========================================================================
uint16_t PacketReader::ExtractParamInt(uint8_t pos)
{
  union{
    unsigned char Buff[2];
    uint16_t d;
  }
  u;

  u.Buff[0] = (unsigned char)m_SBuffer[pos];
  u.Buff[1] = (unsigned char)m_SBuffer[pos+1];
  return(u.d); 
} // ExtractParamInt
