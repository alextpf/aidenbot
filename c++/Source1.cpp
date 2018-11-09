
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

//#include "D:\alex\aidenbot2\c++\VideoProcessor.h"
//#include "D:\alex\aidenbot2\c++\BotManager.h"
//#include "D:\alex\aidenbot2\c++\CheckHSV.h"
//#include "D:\alex\aidenbot2\c++\ImgComposer.h"
#include "C:\Users\alex_\Documents\Arduino\aidenbot\v2\aidenbot\c++\VideoProcessor.h"
#include "C:\Users\alex_\Documents\Arduino\aidenbot\v2\aidenbot\c++\BotManager.h"
#include "C:\Users\alex_\Documents\Arduino\aidenbot\v2\aidenbot\c++\CheckHSV.h"
#include "C:\Users\alex_\Documents\Arduino\aidenbot\v2\aidenbot\c++\ImgComposer.h"

using namespace cv;
using namespace std;

//=======================================================================
bool ReadConfig( std::vector<int> & tmp, const int entries )
{
	string line;

	std::ifstream configFile( "Config.ini" );

	if ( configFile.is_open() )
	{
		for ( int i = 0; i < entries * 2; i++ )
		{
			std::getline( configFile, line );
			if ( i % 2 == 1 )
			{
				const int value = std::stoi( line );
				tmp.push_back( value );
			}
		}
		configFile.close();
	}
	else
	{
		std::cout << "file open error" << std::endl;
		return false;
	}

	return true;
} //ReadConfig

//========================================
int main()
{
	//////////////////
	// variables
	//////////////////

	//char inPath[] = "E:/alex/aidenbot2/data/";
// C:/Users/alex_/Documents/Arduino/aidenbot/v2/aidenbot/data/webcam

	const char inPath[]		= "C:/tmp/data set1/";
	//const char inPath[] = "C:/tmp/result1/";
	//const char inPath[]		= "C:/Users/alex_/Documents/Arduino/aidenbot/v2/aidenbot/data/webcam/";
	const char outPath[]	= "C:/tmp/result1/";
	const char filename[]	= "pic";

	const int webCamId		= 1; // 0: default (laptop's camera), 1: external connected cam
	const int startFrame	= 81;// frame number we want to start at
	const int endFrame		= 890;

	//////////////////////
	// Read from config
	//////////////////////
	std::vector<int> tmp;
	if ( !ReadConfig( tmp, 24 ) ) // read configuration file
	{
		return 0;
	}

	const int operation		= tmp[0]; // 1. run the program, 2. check HSV, 3. record webcam images only
	int inputType			= tmp[1];
	int outputType			= tmp[2];
	const bool showDebugImg	= tmp[3] == 1 ? true : false;
	bool showInputImg		= tmp[4] == 1 ? true : false;
	bool showOutputImg		= tmp[5] == 1 ? true : false;
	const bool manualPickTableCorners	= tmp[6] == 1 ? true : false;
	int delay				= tmp[7];
	const int com			= tmp[8];

	cv::Vec6i red;
	red[0] = tmp[9];	red[1] = tmp[10];	red[2] = tmp[11];	red[3] = tmp[12];	red[4] = tmp[13];	red[5] = tmp[14];

	cv::Vec6i orange;
	orange[0] = tmp[15];	orange[1] = tmp[16];	orange[2] = tmp[17];	orange[3] = tmp[18];	orange[4] = tmp[19];	orange[5] = tmp[20];

	const bool isLog		= tmp[21] == 1 ? true : false;
	const double areaLow	= static_cast<double>( tmp[22] );
	const double areaHigh	= static_cast<double>( tmp[23] );

	switch ( operation )
	{
	case 2: //check hsv
		showInputImg = true;
		showOutputImg = false;
		outputType = 2;
		break;
	case 4: // create resulting imgs by Log.txt
		outputType = 0; // images
		showInputImg = false;
		showOutputImg = false;
		delay = 1;
		break;
	case 5: // create video from images
		inputType = 0; //image
		outputType = 1; // video
		showInputImg = false;
		showOutputImg = false;
		delay = 1;
		break;
	default:
		break;
	}

	char comPort[20];
	sprintf_s( comPort, "\\\\.\\COM%d", com);

	// Create video procesor instance
	VideoProcessor processor;
	BotManager segmentor( comPort );
	CheckHSV hsvChecker;
	ImgComposer imgComposer;

	if ( operation == 1 && inputType == 2 && !segmentor.IsSerialConnected() )
	{
		std::cout << "serial port is not connected" << std::endl; // check "PORT" definition in BotManager.cpp
		return -1;
	}

	segmentor.SetShowDebugImg( showDebugImg );
	segmentor.SetShowOutPutImg( showOutputImg );
	segmentor.SetManualPickTableCorners( manualPickTableCorners );
	segmentor.SetBandWidth(10);
	segmentor.SetRedThreshold( red );
	segmentor.SetOrangeThreshold( orange );
	segmentor.SetIsLog( isLog );
	segmentor.SetPuckAreaThreshLow( areaLow );
	segmentor.SetPuckAreaThreshHigh( areaHigh );

	FrameProcessor * proc = NULL;
	switch ( operation )
	{
	case 1:
		proc = &segmentor;
		break;
	case 2:
		proc = &hsvChecker;
		break;
	case 3:
		break;
	case 4:
		proc = &imgComposer;
		break;
	default:
		break;
	}

	/////////////////////////////////////////////////////
	// Input
	/////////////////////////////////////////////////////
	switch (inputType)
	{
		case 0:
		{
			/////////////////////////
			// input: images
			/////////////////////////
			std::vector<std::string> imgs;

			for (int i = 0; i < endFrame; i++)
			{
				char buffer[100];
				sprintf_s(buffer, "%s%s%03i.jpg", inPath, filename,i);

				std::string name = buffer;
				imgs.push_back(name);
			}

			processor.SetInput(imgs);
		}
		break;

		case 1:
		{
			/////////////////////////
			// input: video
			/////////////////////////
			char buffer[100];
			sprintf_s(buffer, "%s%s.mp4", inPath, filename);

			std::string name = buffer;
			if (!processor.SetInput(name))
			{
				std::cout << "open file error" << std::endl;
			}
		}
		break;

		case 2:
		{
			/////////////////////////
			// input: webcam
			/////////////////////////
			processor.SetInput( webCamId ); //webcam
		}
		break;

		default:
			break;
	} //switch (inputType)

	/////////////////////////////////////////////////////
	// Output
	/////////////////////////////////////////////////////
	switch (outputType)
	{
		case 0:
		{
			/////////////////////////
			// output: images
			/////////////////////////
			char buffer[100];
			sprintf_s(buffer, "%s%s", outPath, filename);

			processor.SetOutput(buffer, ".jpg");
		}
		break;

		case 1:
		{
			/////////////////////////
			// output: video
			/////////////////////////
			char buffer[100];
			sprintf_s(buffer, "%s%s.mp4", outPath, filename );

			int codec = CV_FOURCC( 'D', 'I', 'V', 'X' );
			int fps = 20;
			//int codec = CV_FOURCC( 'P', 'I', 'M', '1' );

			processor.SetOutput( buffer, codec, fps );
		}
		break;

		// case 2:// no output

		default:
			break;
	}//switch (outputType)

	processor.SetFrameProcessor( proc );

	// Declare a window to display the video
	if ( showOutputImg )
	{
		processor.DisplayOutput( "Test Output" );
	}

	// Declare a window to display the input
	if ( showInputImg )
	{
		processor.DisplayInput( "Input" );
	}

	if (!processor.SetFrameNumber(startFrame))
	{
		std::cout << "err";
		return -1;
	}

	processor.SetDelay(delay);
	processor.SetDownSampleRate(1);

	// Start the Process
	processor.Run();

	return 0;
}