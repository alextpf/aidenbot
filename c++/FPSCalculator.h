#pragma once
#include <list>

class FPSCalculator
{
public:
	FPSCalculator();

	int GetFPS();
	void AddFrameTime( const int dt /*ms*/ );
	void SetBufferSize( const int m )
	{
		m_BufferSize = m;
	}

private:
	std::list<int>		m_FrameTimeBuffer;
	int					m_BufferSize;
};//FPSCalculator
