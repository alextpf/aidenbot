#pragma once

#include <list>

class FPSCalculator
{
public:
	float GetFPS( const int dt /*ms*/ );
	void SetBufferSize( const int m )
	{
		m_BufferSize = m;
	}
private:
	std::list<int>		m_FrameTimeBuffer;
	int					m_BufferSize;
};//FPSCalculator
