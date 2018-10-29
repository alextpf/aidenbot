#include "FPSCalculator.h"
//=============================================
FPSCalculator::FPSCalculator()
	: m_BufferSize( 0 )
{}

//=============================================
int FPSCalculator::GetFPS()
{
	float s = static_cast<float>( m_FrameTimeBuffer.size() );
	
	float sum( 0 );

	std::list<int>::iterator it = m_FrameTimeBuffer.begin();
	
	for ( int i = 0; i < s; i++, it++ )
	{
		sum += static_cast<float>( *it );
	}
	
	int fps = static_cast<int>( s / sum * 1000.0f + 0.5f );

	return fps;
}

//=============================================
void FPSCalculator::AddFrameTime( const int dt /*ms*/ )
{
	int s = static_cast<int>( m_FrameTimeBuffer.size() );

	if ( s >= m_BufferSize )
	{
		m_FrameTimeBuffer.pop_front();
	}

	m_FrameTimeBuffer.push_back( dt );

}// AddFrameTime