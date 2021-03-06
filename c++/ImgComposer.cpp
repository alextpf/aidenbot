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
#define ORANGE cv::Scalar( 0, 128, 255 )

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
    std::string puckSpeed;
    std::string puckAvgSpeed;
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
	stream >> puckPos.y;

	cv::Point bouncePos( 0, 0 );
	cv::Point predPos( 0, 0 );
	cv::Point prevPos( 0, 0 );
	cv::Point botPos( 0, 0 ); // desired pos
	cv::Point detectedBotPos( 0, 0 ); // detected pos

	bool hasBounce( false );
	int predictStatus( -1 );

	const bool puckFound = puckPos.x != -1;

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

	// detected bot position
	std::getline( log, line ); // get description, skip
	std::getline( log, line );
	stream.str( line );
	stream.seekg( 0 );

	stream >> detectedBotPos.x;
	stream >> detectedBotPos.y;

	// current puck speed
	std::getline( log, puckSpeed ); // get description, skip
	std::getline( log, puckSpeed );

	// predictTimeDefence
	std::getline( log, line ); // get description, skip
	std::getline( log, line );

	// predictTimeAttack
	std::getline( log, line ); // get description, skip
	std::getline( log, line );

    // predictTimeAtBounce
    std::getline( log, line ); // get description, skip
    std::getline( log, line );

	// predict numBounce
	std::getline( log, line ); // get description, skip
	std::getline( log, line );

	// desired bot X/Y speed
	std::getline( log, line ); // get description, skip
	std::getline( log, line );

	// predict status
	std::getline( log, line ); // get description, skip
	std::getline( log, line );
	predictStatus = stoi( line );

	// bot status
	std::getline( log, line ); // get description, skip
	std::getline( log, line );
	int botStatus = stoi( line );

	// attack status
	std::getline( log, line ); // get description, skip
	std::getline( log, line );
	int attackStatus = stoi( line );

	// attack time
	std::getline( log, line ); // get description, skip
	std::getline( log, line );

	// puck average speed
	std::getline( log, puckAvgSpeed ); // get description, skip
	std::getline( log, puckAvgSpeed );

    // correct missing steps
    std::getline( log, line ); // get description, skip
    std::getline( log, line );

	// bail out?
	std::getline( log, line ); // get description, skip
	std::getline( log, line );

	/////////////////
	// Draw now
	/////////////////

    // draw table. m_Corners order: tl, tr, ll, lr
	cv::line( output, m_Corners[0], m_Corners[1], GREEN, 2 );
	cv::line( output, m_Corners[0], m_Corners[2], GREEN, 2 );
	cv::line( output, m_Corners[1], m_Corners[3], GREEN, 2 );
	cv::line( output, m_Corners[2], m_Corners[3], GREEN, 2 );

	// show frame number on screen
	std::string text = "#" + std::to_string( frameNum );
	cv::Point origin( 30, 25 ); // upper left
	int thickness = 1;
	int lineType = 8;

	cv::putText( output, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, ORANGE, thickness, lineType );

    // show puck speed speed on screen
    text = "Puck Speed: " + puckSpeed;
    origin = cv::Point( 30, 40 );

	cv::putText( output, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, ORANGE, thickness, lineType );

	// show puck avg speed speed on screen
    text = "Puck Avg Speed: " + puckAvgSpeed;
    origin = cv::Point( 30, 55 );

    cv::putText( output, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, ORANGE, thickness, lineType );

	// show frame time on screen
	text = "Frame time: " + std::to_string( frameTime ) + " ms";
	origin = cv::Point( 30, 70 );

	cv::putText( output, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, ORANGE, thickness, lineType );

	// show FPS on screen
	text = "FPS = " + std::to_string( fps );
	origin = cv::Point( 520, 25 ); // upper right

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

		// desired bot pos
		radius = 5;
		cv::circle( output, botPos, radius, BLUE, thickness );

	} // if ( puckFound )

	// detected bot pos
	int radius = 12;
	cv::circle( output, detectedBotPos, radius, RED, thickness );

	///////////////////////////////////////////////////////////
	filePos = static_cast<int>( log.tellg() );
	log.close();

} // Process