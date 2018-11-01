#include "PacketReader.h"
#include "Configuration.h"

//#define DEBUG_SERIAL
//==========================================================================
PacketReader::PacketReader()
    : m_DesiredMotorSpeed( MAX_ABS_SPEED )
    , m_IsPacketRead( false )
{}

//==========================================================================
bool PacketReader::ReadPacket2()
{
    const byte numBytes = 12; // 10 byte data + 2 bytes sync markers
    static bool inSync = false; // true: ready; false: not ready
    static byte counter = 0;
    
    byte startMarker = 0x41; // 'A'
    byte endMarker = 0x42; // 'B'

    byte tmp;
    bool ret = false;
    
    if ( Serial.available() > 0 )
    {
        // We rotate the Buffer (we could implement a ring buffer in future)
        for( int i = numBytes - 1 ; i > 0 ; i-- )
        {
            m_Buffer[i] = m_Buffer[i - 1];
        }
#ifdef DEBUG_SERIAL        
        //debug
        int i = Serial.available();
        Serial.print("# available=");
        Serial.println(i);
#endif
        
        m_Buffer[0] = Serial.read();

#ifdef DEBUG_SERIAL        
        Serial.print("m_Buffer[0] = ");
        Serial.println(m_Buffer[0]);
#endif
        
        // We look for a  message start like "AA" to sync packets
        if( ( m_Buffer[0] == startMarker ) && ( m_Buffer[1] == startMarker ) )
        {
            if( inSync )
            {
                Serial.println( "S ERR" );
            }
            
            inSync = true;
            counter = numBytes-2;
        }
        else if( inSync )
        {
#ifdef DEBUG_SERIAL          
            Serial.print("counter=");
            Serial.println(counter);
#endif
            
            counter--;   // Until we complete the packet
            
            if( counter <= 0 )   // packet complete!!
            {
#ifdef DEBUG_SERIAL              
                Serial.println("done");
#endif                
              
                // Extract parameters
                m_DesiredBotPos.m_X = ExtractParamInt( 8 );
                m_DesiredBotPos.m_Y = ExtractParamInt( 6 );
                m_DetectedBotPos.m_X = ExtractParamInt( 4 );
                m_DetectedBotPos.m_Y = ExtractParamInt( 2 );
                m_DesiredMotorSpeed = ExtractParamInt( 0 );
                
                inSync = false;
                m_IsPacketRead = true;

                ret = true;
            }
        }
    }    
    return ret;
} // ReadPacket2

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
        for (byte n = 0; n < 12; n++) {
            Serial.print(m_Buffer[n], HEX);
            Serial.print(' ');
        }
        Serial.println();
        m_IsPacketRead = false;
    }
}
