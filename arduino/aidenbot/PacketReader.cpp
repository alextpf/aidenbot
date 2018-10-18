#include "PacketReader.h"

//==========================================================================
PacketReader::PacketReader()
    : m_MsgFullyRead( false )
    , m_ReadCounter( 0 )
{}

//==========================================================================
void PacketReader::ReadPacket()
{
    if( Serial.available() > 0 )
    {
        //Serial.println("P");
        // We rotate the Buffer (we could implement a ring buffer in future)
        for( int i = 11; i > 0; i-- )
        {
            m_SBuffer[i] = m_SBuffer[i - 1];
        }

        m_SBuffer[0] = Serial.read();
        //Serial.print(S1Buffer[0]);
        // We look for a  message start like "AA" to sync packets
        if( ( m_SBuffer[0] == 'A' ) && ( m_SBuffer[1] == 'A' ) )
        {
            if( !m_MsgFullyRead )
            {
                m_MsgFullyRead = true;                
            }
            else
            {
                Serial.println( "S ERR" );
            }

            m_ReadCounter = 12;

            return;
        }
        else if( m_MsgFullyRead )
        {
            m_ReadCounter--;   // Until we complete the packet
            if( m_ReadCounter <= 0 )   // packet complete!!
            {
                // Extract parameters
                puckPixX = extractParamInt( 8 );
                puckPixY = extractParamInt( 6 );
                puckSize = extractParamInt( 4 );
                robotPixX = extractParamInt( 2 );
                robotPixY = extractParamInt( 0 );
                m_MsgFullyRead = false;
            }
        }
    }
}
