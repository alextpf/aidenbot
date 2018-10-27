#include "FPSCalculator.h"

//=============================================
float FPSCalculator::GetFPS( const int dt /*ms*/ )
{
	int s = static_cast<int>( m_FrameTimeBuffer.size() );
	
	if ( s >= m_BufferSize )
	{
		m_FrameTimeBuffer.pop_front();
	}
	else
	{
		s++;
	}

	m_FrameTimeBuffer.push_back( dt );

	float sum( 0 );

	std::list<int>::iterator it = m_FrameTimeBuffer.begin();
	
	for ( int i = 0; i < s; i++, it++ )
	{
		sum += static_cast<float>( *it );
	}

	float fps = sum / s;

	return fps;
}