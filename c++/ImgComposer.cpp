#include "ImgComposer.h"
#include <iostream>
#include <fstream>
//===============================================================
void ImgComposer::Process( cv::Mat & input, cv::Mat & output )
{
	std::ifstream log;
	log.open( "Log.txt", std::ifstream::in );
	if ( !log )
	{
		std::cout << "error opening log" << std::endl;
		return;
	}

	// clone input to output
	output = input.clone();

	std::string line;
	std::stringstream stream;

	static bool tableRead = false;
	if ( !tableRead )
	{
		std::getline( log, line ); // get description, skip

		for ( int i = 0; i < 4; i++ )
		{
			std::getline( log, line ); // get numbers, now parse them
			stream.str( line );

			cv::Point pt;
			stream >> pt.x;
			stream >> pt.y;
			m_Corners.push_back( pt );
		}
	}

	//frame number
	std::getline( log, line ); // get description, skip
	std::getline( log, line );
	int frameNum = std::stoi( line );

	//farme time
	std::getline( log, line ); // get description, skip
	std::getline( log, line );
	int frameTime = std::stoi( line );

	// fps
	std::getline( log, line ); // get description, skip
	std::getline( log, line );
	int fps = std::stoi( line );

	// puck pos
	std::getline( log, line ); // get description, skip
	std::getline( log, line );
	stream.str( line );
	cv::Point puckPos( 0, 0 );
	stream >> puckPos.x;

	if ( puckPos.x == -1 )
	{
		return;
	}

	log.close();

} // Process