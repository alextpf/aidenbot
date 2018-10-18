class PacketReader
{
public:
    PacketReader();

    void ReadPacket();
private:

    char    m_SBuffer[12];
    bool    m_MsgFullyRead; // true: ready; false: not ready
    int     m_ReadCounter;
}; // PacketReader
