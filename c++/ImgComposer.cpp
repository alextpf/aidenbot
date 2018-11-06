#include "ImgComposer.h"
#include <iostream>
#include <fstream>

// color definition
#define BLUE   cv::Scalar( 255,   0,   0 ) //BGR
#define GREEN  cv::Scalar(   0, 255,   0 )
#define RED    cv::Scalar(   0,   0, 255 )
#define YELLOW cv::Scalar(   0, 255, 255 )
#define WHITE  cv::Scalar( 255, 255, 255 )
#define BLACK  cv::Scalar(   0,   0,   0 )
#define PURPLE cv::Scalar( 255, 112, 132 )
#define MEDIUM_PURPLE cv::Scalar( 219, 112, 147 )

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
	static int filePos = 0;
	log.seekg( filePos );

	static bool tableRead = false;
	if ( !tableRead )
	{
		std::getline( log, line ); // get description, skip

		for ( int i = 0; i < 4; i++ )
		{
			std::getline( log, line ); // get numbers, now parse them
			stream.str( line );
			stream.seekg( 0 );

			cv::Point pt;
			stream >> pt.x;
			stream >> pt.y;
			m_Corners.push_back( pt );
		}

		tableRead = true;
	}//if ( !tableRead )

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
	stream.seekg( 0 );
	cv::Point puckPos( 0, 0 );
	stream >> puckPos.x;

	cv::Point bouncePos( 0, 0 );
	cv::Point predPos( 0, 0 );
	cv::Point prevPos( 0, 0 );
	cv::Point botPos( 0, 0 );
	bool hasBounce;
	int predictStatus( -1 );

	const bool puckFound = puckPos.x != -1;
	if (puckFound )
	{
		stream >> puckPos.y;

		// bounce position
		std::getline( log, line ); // get description, skip
		std::getline( log, line );
		stream.str( line );
		stream.seekg( 0 );

		stream >> bouncePos.x;
		stream >> bouncePos.y;

		hasBounce = bouncePos.x != -1;

		// predicted position
		std::getline( log, line ); // get description, skip
		std::getline( log, line );
		stream.str( line );
		stream.seekg( 0 );

		stream >> predPos.x;
		stream >> predPos.y;

		// previous position
		std::getline( log, line ); // get description, skip
		std::getline( log, line );
		stream.str( line );
		stream.seekg( 0 );

		stream >> prevPos.x;
		stream >> prevPos.y;

		// desired bot position
		std::getline( log, line ); // get description, skip
		std::getline( log, line );
		stream.str( line );
		stream.seekg( 0 );

		stream >> botPos.x;
		stream >> botPos.y;

		// desired bot speed
		std::getline( log, line ); // get description, skip
		std::getline( log, line );
		int speed = stoi( line );

		// predict status
		std::getline( log, line ); // get description, skip
		std::getline( log, line );
		predictStatus = stoi( line );

		// bot status
		std::getline( log, line ); // get description, skip
		std::getline( log, line );
		int botStatus = stoi( line );
	} // if ( puckPos.x == -1 )

	/////////////////
	// Draw now
	/////////////////

    // draw table. m_Corners order: tl, tr, ll, lr
	cv::line( output, m_Corners[0], m_Corners[1], GREEN, 2 );
	cv::line( output, m_Corners[0], m_Corners[2], GREEN, 2 );
	cv::line( output, m_Corners[1], m_Corners[3], GREEN, 2 );
	cv::line( output, m_Corners[2], m_Corners[3], GREEN, 2 );

	// show FPS on screen
	const std::string text = "FPS = " + std::to_string( fps );
	cv::Point origin( 520, 25 ); // upper right
	int thickness = 1;
	int lineType = 8;

	cv::putText( output, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, MEDIUM_PURPLE, thickness, lineType );

	if ( puckFound )
	{
		// draw puck contour
		int radius = 15;
		const int thickness = 2;
		cv::circle( output, puckPos, radius, GREEN, thickness );

		// draw previous pos
		cv::line( output, prevPos, puckPos, cv::Scalar( 130, 221, 238 ), 2 );

		if ( predictStatus == 1 || predictStatus == 2 )
		{
			// draw bounce point and predicted point
			if ( hasBounce )
			{
				// have one bounce
				cv::line( output, bouncePos, puckPos, PURPLE, 2 );
				cv::line( output, predPos, bouncePos, PURPLE, 2 );
			}
			else
			{
				// no bounce
				cv::line( output, predPos, puckPos, PURPLE, 2 );
			} // if ( hasBounce )

		} // if ( predictStatus == 1 || predictStatus == 2 )

		radius = 5;
		cv::circle( output, botPos, radius, BLUE, thickness );
	} // if ( puckFound )

	///////////////////////////////////////////////////////////
	filePos = static_cast<int>( log.tellg() );
	log.close();

} // Process