#include "BotManager.h"
#include "DiskFinder.h"
#include "Utility.h"

#define PI							3.1415926
#define DEG_TO_RAD					PI / 180.0f

// color definition
#define BLUE   cv::Scalar(255, 0, 0) //BGR
#define GREEN  cv::Scalar(0, 255, 0)
#define RED    cv::Scalar(0, 0, 255)
#define YELLOW cv::Scalar(0, 255, 255)
#define WHITE  cv::Scalar(255, 255, 255)
#define BLACK  cv::Scalar(0,0,0)

#define PORT "\\\\.\\COM4"

#define DEBUG_SERIAL

//=======================================================================
// helper function to show type
std::string type2str(int type)
{
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}//std::string type2str(int type)

//=======================================================================
BotManager::BotManager()
: m_TableFound( false )
, m_BandWidth( 0 )
, m_SerialPort( PORT )
, m_ShowDebugImg( false )
, m_ShowOutPutImg( true )
, m_ManualPickTableCorners( false )
{}

//=======================================================================
void BotManager::Process(cv::Mat & input, cv::Mat & output)
{
	if ( m_ShowOutPutImg )
	{
		output = input.clone();
	}

	// find table range
	char windowName[] = "corners";

	if ( !m_TableFound )
	{
		cv::Point TopLeft;
		cv::Point TopRight;
		cv::Point LowerLeft;
		cv::Point LowerRight;

		// user-picked 4 corners
		cv::imshow( windowName, input );
		cv::setMouseCallback( windowName, OnMouse, &m_Corners );
		while ( m_Corners.size() < 4 )
		{
			size_t m = m_Corners.size();
			if ( m > 0 )
			{
				cv::circle( input, m_Corners[m - 1], 3, GREEN, 2 );
			}

			cv::imshow( windowName, input );
			cv::waitKey( 10 );
		}

		// last point
		cv::circle( input, m_Corners[3], 3, GREEN, 2 );

		cv::imshow( windowName, input );
		cv::waitKey( 10 );
		// order the 4 corners
		OrderCorners();

		if ( m_ShowDebugImg )
		{
			// draw the bands around 4 picked corners
			// m_Corners is arranged by: ul, ur, ll, lr
			cv::circle( input, m_Corners[0], 3, GREEN, 2 );
			cv::circle( input, m_Corners[1], 3, RED, 2 );
			cv::circle( input, m_Corners[2], 3, BLUE, 2 );
			cv::circle( input, m_Corners[3], 3, WHITE, 2 );

			cv::imshow( "ORder:", input );
		} // DEBUG

		if ( !m_ManualPickTableCorners )
		{
			// blur image first by Gaussian
			int kernelSize = 3;
			double std = 2.0;

			cv::GaussianBlur( input, input, cv::Size( kernelSize, kernelSize ), std, std );

			if ( m_ShowDebugImg )
			{
				cv::imshow( "gauss:", input );
			} // DEBUG

			// convert to gray scale
			cv::Mat tmp;
			cv::cvtColor( input, tmp, cv::COLOR_RGB2GRAY );

			cv::Mat tmp2 = tmp.clone();

			// adaptive threshold
			int adaptiveMethod = cv::ADAPTIVE_THRESH_MEAN_C;
			int thresholdType = cv::THRESH_BINARY;
			int blockSiz = 55;

			cv::adaptiveThreshold( tmp2, tmp2, 255, adaptiveMethod, thresholdType, blockSiz, 5 );
			//cv::threshold(tmp, tmp1, 128, 255, cv::THRESH_BINARY_INV);

			// close, fill the "holes" in the foreground
			//cv::dilate( tmp2, tmp2, cv::Mat(), cv::Point( -1, -1 ), 2/*num iteration*/ );
	  //      cv::erode( tmp2, tmp2, cv::Mat(), cv::Point( -1, -1 ), 2/*num iteration*/ );

			cv::Mat ellipse = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 5, 5 ) );
			cv::morphologyEx( tmp2, tmp2, cv::MORPH_CLOSE, ellipse, cv::Point( -1, -1 ), 1/*num iteration*/ );

			if ( m_ShowDebugImg )
			{
				cv::imshow( "adaptiveThreshold + closing:", tmp2 );
			} // DEBUG

			std::vector< std::vector< cv::Point > > contours;
			std::vector< cv::Vec4i > hierarchy;
			cv::findContours( tmp2, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE );

			//find the contour that's greater than some sizes, i.e. the table
			double thresh = tmp2.size().width * tmp2.size().height * 0.5;

			std::vector< std::vector< cv::Point > > leftOver;
			for ( int i = 0; i < contours.size(); i++ )
			{
				double area = cv::contourArea( contours[i] );
				if ( area > thresh )
				{
					leftOver.push_back( contours[i] );
				}
			}

			if ( leftOver.size() == 1 )
			{
				tmp2 = cv::Mat::zeros( tmp2.size(), CV_8UC1 );
				drawContours( tmp2, leftOver, 0, 255/*color*/, cv::FILLED );
				// do closing to clean noise again
				cv::morphologyEx( tmp2, tmp2, cv::MORPH_CLOSE, ellipse, cv::Point( -1, -1 ), 1/*num iteration*/ );

				if ( m_ShowDebugImg )
				{
					cv::imshow( "contour:", tmp2 );
				} // DEBUG
			}
			else
			{
				// threshold failed, bail out
				tmp2 = tmp.clone();
			}

			// canny low & heigh threshold
			int low = 50;
			int high = 100;

			cv::Canny( tmp2, tmp2, low, high );

			// dilate Canny results
			cv::dilate( tmp2, tmp2, cv::Mat(), cv::Point( -1, -1 ), 2 /*num iteration*/ );

			if ( m_ShowDebugImg )
			{
				cv::imshow( "canny:", tmp2 );
			} // DEBUG

			// mask out anything that's outside of the user-picked band
			MaskCanny( tmp2 );

			if ( m_ShowDebugImg )
			{
				cv::Mat copy = input.clone();

				// draw bounds
				cv::Scalar color = GREEN; // green

				cv::line( copy, m_o_ul, m_o_ur, color );
				cv::line( copy, m_o_ul, m_o_ll, color );
				cv::line( copy, m_o_ur, m_o_lr, color );
				cv::line( copy, m_o_ll, m_o_lr, color );

				cv::line( copy, m_i_ul, m_i_ur, color );
				cv::line( copy, m_i_ul, m_i_ll, color );
				cv::line( copy, m_i_ur, m_i_lr, color );
				cv::line( copy, m_i_ll, m_i_lr, color );

				cv::imshow( "bound:", copy );
			} // DEBUG

			// Hough line transform
			double dRho = 1.0f;
			double dTheta = CV_PI / 180.0f;
			unsigned int minVote = 80;//360 / 2;
			float minLength = 50.0f;// 360 / 2;
			float maxGap = 10.0f;
			//LineFinder::METHOD m = LineFinder::METHOD::TRAD;
			LineFinder::METHOD m = LineFinder::METHOD::PROB;

			m_TableFinder.SetMethod( m );
			m_TableFinder.SetDeltaRho( dRho );
			m_TableFinder.SetDeltaTheta( dTheta );
			m_TableFinder.SetMinVote( minVote );
			m_TableFinder.SetMinLength( minLength );
			m_TableFinder.SetMaxGap( maxGap );

			if ( m == LineFinder::METHOD::PROB )
			{
				const std::vector<cv::Vec4i>& lines = m_TableFinder.FindLinesP( tmp2 );

				if ( m_ShowDebugImg )
				{
					//cv::Mat copy = input.clone();
					m_TableFinder.DrawDetectedLines( input, BLUE );
					cv::imshow( "HoughLine:", input );
				} // DEBUG

				// filter out the lines that's out of bound
				if ( !m_TableFinder.Refine4Edges( m_Corners, m_BandWidth, input/*debug use*/ ) )
				{
					std::cout << "error" << std::endl;
					return;
				}
			}
			else
			{
				const std::vector<cv::Vec2f>& lines = m_TableFinder.FindLines( tmp2 );
			}

			if ( m_ShowDebugImg )
			{
				m_TableFinder.DrawTableLines( input, RED );
				cv::imshow( "Filtered HoughLine:", input );
			} // DEBUG

			// 4 corners
			TopLeft = m_TableFinder.GetTopLeft();
			TopRight = m_TableFinder.GetTopRight();
			LowerLeft = m_TableFinder.GetLowerLeft();
			LowerRight = m_TableFinder.GetLowerRight();

		}
		else
		{
			// corners is arranged by: ul, ur, ll, lr
			TopLeft = m_Corners[0];
			TopRight = m_Corners[1];
			LowerLeft = m_Corners[2];
			LowerRight = m_Corners[3];
		} // if ( !m_ManualPickTableCorners )

		// construct mask based on table 4 corners
		std::vector< cv::Point > tmpContour;
		tmpContour.push_back( TopRight );
		tmpContour.push_back( TopLeft );
		tmpContour.push_back( LowerLeft );
		tmpContour.push_back( LowerRight );

		std::vector< std::vector< cv::Point > > tableContour;
		tableContour.push_back( tmpContour );
		m_Mask = cv::Mat::zeros( input.size(), CV_8UC1 );
		drawContours( m_Mask, tableContour, 0, 255/*color*/, cv::FILLED );

		if ( m_ShowDebugImg )
		{
			cv::imshow( "Mask:", m_Mask );
		} // DEBUG

		m_TableFound = true;
	} // if ( !m_TableFound )

	if ( m_TableFound )
	{
		cv::destroyWindow( windowName );

		if ( m_ShowOutPutImg )
		{
			// draw table boundary
			cv::Point TopLeft( static_cast<int>( m_TableFinder.GetLeft() ), static_cast<int>( m_TableFinder.GetTop() ) );
			cv::Point TopRight( static_cast<int>( m_TableFinder.GetRight() ), static_cast<int>( m_TableFinder.GetTop() ) );
			cv::Point LowerLeft( static_cast<int>( m_TableFinder.GetLeft() ), static_cast<int>( m_TableFinder.GetBottom() ) );
			cv::Point LowerRight( static_cast<int>( m_TableFinder.GetRight() ), static_cast<int>( m_TableFinder.GetBottom() ) );

			cv::line( output, TopLeft, TopRight, GREEN, 2 );
			cv::line( output, TopLeft, LowerLeft, GREEN, 2 );
			cv::line( output, TopRight, LowerRight, GREEN, 2 );
			cv::line( output, LowerLeft, LowerRight, GREEN, 2 );
		}

		// find puck and robot position
        DiskFinder diskFinder;
        Contours contours;
        cv::Point center;

        // convert RBG to HSV first
        cv::Mat hsvImg;
        cv::cvtColor( input, hsvImg, CV_BGR2HSV );

		// 1. find puck
		cv::Vec6i redThresh( 160, 110, 110, 179, 255, 220 ); // lowH, lowS, lowV, highH, highS, highV
		cv::Vec6i orangeThresh( 0, 110, 110, 20, 255, 220 ); // lowH, lowS, lowV, highH, highS, highV

		const bool hasPuck = diskFinder.FindDisk2Thresh(
			contours, center, hsvImg, redThresh, orangeThresh, m_Mask );
		//draw puck center
		if ( m_ShowOutPutImg )
		{
			const int radius = 10;
			const int thickness = 2;
			cv::circle( output, center, radius, GREEN, thickness );
		}

        if( !hasPuck )
        {
            std::cout << "can't find puck" << std::endl;
            return;
        }

        // convert screen coordinate to table coordinate
		const cv::Point puckPos = m_TableFinder.ImgToTableCoordinate( center ); // mm, in table coordinate

		m_Camera.SetCurrPuckPos( puckPos );

        // 2. find robot

        //cv::Vec6i orangeThresh( 0, 110, 110, 20, 255, 220 ); // lowH, lowS, lowV, highH, highS, highV

        //const bool hasRobot = diskFinder.FindDisk1Thresh(
        //    contours, center, hsvImg, redThresh, m_Mask );

        //if( !hasRobot )
        //{
        //    std::cout << "can't find Robot" << std::endl;
        //    return;
        //}

        //// convert screen coordinate to table coordinate
        //const cv::Point robotPos = m_TableFinder.ImgToTableCoordinate( center ); // mm, in table coordinate

        //m_Camera.SetRobotPos( robotPos );

        // get time stamp
		clock_t curr = clock();

		int dt = static_cast<int>( ( curr - m_CurrTime ) * 1000.0f / CLOCKS_PER_SEC ); // in ms

        // skip processing if 1st frame
		if ( /*dt < 2000 &&*/ m_CurrTime > 0)
		{
            // do prediction work
			m_Camera.CamProcess( dt );

            // determine robot strategy
			m_Robot.NewDataStrategy( m_Camera );

            // determine robot position
            m_Robot.RobotMoveDecision( m_Camera );

            // send the message by com port over to Arduino
            SendMessage();
#ifdef DEBUG_SERIAL
            ReceiveMessage();
#endif // DEBUG

		}

        // update time stamp and puck position
        m_CurrTime = curr;
		m_Camera.SetPrevPuckPos( puckPos );
	} // if ( m_TableFound )

}//Process

//=======================================================================
void BotManager::OnMouse( int event, int x, int y, int f, void* data )
{
	std::vector<cv::Point> *curobj = reinterpret_cast<std::vector<cv::Point>*>( data );

	if ( event == cv::EVENT_LBUTTONDOWN )
	{
		curobj->push_back( cv::Point( x, y ) );
	}
}//OnMouse

//=======================================================================
void BotManager::SendMessage()
{
	// message lay out :
	// 0, 1: for sync use, "AA"
	// 2, 3: desired robot pos X
	// 4, 5: desired robot pos Y
	// 6, 7: detected robot pos X
	// 8, 9: detected robot pos Y
	// 10, 11: desired speed
	BYTE message[12];

	// Initial sync "AA"
	message[0] = 0x41;
	message[1] = 0x41;

	// table size is 1003 x 597, which is within 2 bytes (2^16 = 65536, 2^8 = 256)
	// so we use 2 bytes to store position. Similarly for speed

	// desired robot pos
	cv::Point desiredBotPos = m_Robot.GetDesiredRobotPos();

	// Pos X (high byte, low byte)
	message[2] = ( desiredBotPos.x >> 8 ) & 0xFF;
	message[3] = desiredBotPos.x & 0xFF;

	// Pos Y (high byte, low byte)
	message[4] = ( desiredBotPos.y >> 8 ) & 0xFF;
	message[5] = desiredBotPos.y & 0xFF;

	// detected robot pos
	cv::Point detectedBotPos = m_Camera.GetRobotPos();

	// Pos X (high byte, low byte)
	message[6] = ( detectedBotPos.x >> 8 ) & 0xFF;
	message[7] = detectedBotPos.x & 0xFF;

	// Pos Y (high byte, low byte)
	message[8] = ( detectedBotPos.y >> 8 ) & 0xFF;
	message[9] = detectedBotPos.y & 0xFF;

	// speed
	int speed = m_Robot.GetDesiredRobotSpeed();
	message[10] = ( speed >> 8 ) & 0xFF;
	message[11] = speed & 0xFF;

    m_SerialPort.WriteSerialPort<BYTE>( message, 12 );
} // SendMessage

//=======================================================================
void BotManager::ReceiveMessage()
{
    char msg[200];

    int res = m_SerialPort.ReadSerialPort<char>( msg, 200 );
    std::cout << msg << std::endl;

} // ReceiveMessage

//=======================================================================
void BotManager::OrderCorners()
{
	if (m_Corners.size() != 4)
	{
		return;
	}

	cv::Point left1(100000, 0); // left most
	cv::Point left2(100000, 0); // 2nd to left most
	cv::Point right1(-1, 0); // right most
	cv::Point right2(-1, 0); // 2nd to right most

	for (int i = 0; i < 4; i++)
	{
		if (m_Corners[i].x < left1.x)
		{
			left2 = left1;
			left1 = m_Corners[i];
		}
		else if (m_Corners[i].x < left2.x)
		{
			left2 = m_Corners[i];
		}

		if (m_Corners[i].x > right1.x)
		{
			right2 = right1;
			right1 = m_Corners[i];
		}
		else if (m_Corners[i].x > right2.x)
		{
			right2 = m_Corners[i];
		}
	} // for i = 1 - 4

	// determine upper and lower
	// m_Corners is arranged by: ul, ur, ll, lr

	if (left1.y > left2.y)
	{
		// left1 is lower left, left2 is upper left
		m_Corners[0] = left2;
		m_Corners[2] = left1;
	}
	else
	{
		// left2 is lower left, left1 is upper left
		m_Corners[0] = left1;
		m_Corners[2] = left2;
	}

	if (right1.y > right2.y)
	{
		// right1 is lower right, right2 is upper right
		m_Corners[1] = right2;
		m_Corners[3] = right1;
	}
	else
	{
		// right2 is lower right, right1 is upper right
		m_Corners[1] = right1;
		m_Corners[3] = right2;
	}

}// OrderCorners

//=======================================================================
void BotManager::MaskCanny(cv::Mat & img)
{
    // Generate a band around user-picked 4 corners.
    // This band is wider than the m_BandWidth because
    // such that later in Hough transform, it doesn't
    // detect the artificial line generated by the masking.
    // After Hough transform line detection, we then
    // mask again using the true m_BandWidth
    cv::Size s = img.size();

    float o_l, o_r, o_t, o_b;
    float i_l, i_r, i_t, i_b;

    const unsigned int offset = m_BandWidth + 10;

    Utility::GenerateBand(
        o_l, o_r, o_t, o_b,
        i_l, i_r, i_t, i_b,
        m_o_ul, m_o_ur, m_o_ll, m_o_lr,
        m_i_ul, m_i_ur, m_i_ll, m_i_lr,
        m_Corners, offset, offset );

	for (unsigned int i = 0; i < static_cast<unsigned int>(s.width); i++)
	{
		for (unsigned int j = 0; j < static_cast<unsigned int>(s.height); j++)
		{
			const bool isOutsideOuter = Utility::IsOutsideOuter( i, j, o_l, o_r, o_t, o_b, m_o_ur, m_o_ll, m_o_lr );
			if ( isOutsideOuter )
			{
				img.at<uchar>(j, i) = 0;
			}
			else
			{
				const bool isInsideInner = Utility::IsInsideInner(i, j, i_l, i_r, i_t, i_b, m_i_ur, m_i_ll, m_i_lr );
				if ( isInsideInner )
				{
					img.at<uchar>(j, i) = 0;
				}
			}
		}
	}

	if ( m_ShowDebugImg )
	{
		cv::imshow( "masked canny:", img );
	} // DEBUG

}//void MaskCanny(cv::Mat & img);

//=======================================================================
void BotManager::SetShowDebugImg( const bool ok )
{
	m_ShowDebugImg = ok;
} // SetShowDebugImg

//=======================================================================
void BotManager::SetManualPickTableCorners( const bool ok )
{
	m_ManualPickTableCorners = ok;
} // SetManualPickTableCorners

//=======================================================================
void BotManager::SetShowOutPutImg( const bool ok )
{
	m_ShowOutPutImg = ok;
} // SetShowOutPutImg