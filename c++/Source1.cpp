
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

//#include "E:\alex\aidenbot2\c++\VideoProcessor.h"
//#include "E:\alex\aidenbot2\c++\BotManager.h"
//#include "E:\alex\aidenbot2\c++\CheckHSV.h"
#include "C:\Users\alex_\Documents\Arduino\aidenbot\v2\aidenbot\c++\VideoProcessor.h"
#include "C:\Users\alex_\Documents\Arduino\aidenbot\v2\aidenbot\c++\BotManager.h"
#include "C:\Users\alex_\Documents\Arduino\aidenbot\v2\aidenbot\c++\CheckHSV.h"

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
	std::vector<int> tmp;
	if ( !ReadConfig( tmp, 9 ) ) // read configuration file
	{
		return 0;
	}

	int operation		= tmp[0]; // 1. run the program, 2. check HSV, 3. record webcam images only
	int inputType		= tmp[1];
	int outputType		= tmp[2];
	bool showDebugImg	= tmp[3] == 1 ? true : false;
	bool showInputImg	= tmp[4] == 1 ? true : false;
	bool showOutputImg  = tmp[5] == 1 ? true : false;
	bool manualPickTableCorners = tmp[6] == 1 ? true : false;
	int delay			= tmp[7];
	int com				= tmp[8];

	char comPort[20];
	sprintf_s( comPort, "\\\\.\\COM%d", com);

	// Create video procesor instance
	VideoProcessor processor;
	BotManager segmentor( comPort );
	CheckHSV hsvChecker;

	if ( !segmentor.IsSerialConnected() )
	{
		std::cout << "serial port is not connected" << std::endl; // check "PORT" definition in BotManager.cpp
		return -1;
	}

	segmentor.SetShowDebugImg( showDebugImg );
	segmentor.SetShowOutPutImg( showOutputImg );
	segmentor.SetManualPickTableCorners( manualPickTableCorners );
	segmentor.SetBandWidth(10);

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
	default:
		break;
	}

	//////////////////
	// variables
	//////////////////

	//char path[] = "E:/alex/aidenbot2/data/";
	char path[] = "C:/Users/alex_/Documents/Arduino/aidenbot/v2/aidenbot/data/webcam/";
	char filename[] = "vid";

	int webCamId = 1; // 0: default (laptop's camera), 1: external connected cam

	// note: 213 - 380 good samples
    // 452 - 507 samples for attack
	// 653 - 700
	//int num =  755; //249;
	//int startFrame =  754;//248;// frame number we want to start at
	int num = 719;
	int startFrame = 716;// frame number we want to start at
	//int num = 184;
	//int startFrame = 183;// frame number we want to start at
	//char inputMode[] = "";
	char outputMode[] = "";

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

			for (int i = 0; i < num; i++)
			{
				char buffer[100];
				sprintf_s(buffer, "%s%s%03i.jpg", path, filename,i);

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
			sprintf_s(buffer, "%s%s.mp4", path, filename);

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
			sprintf_s(buffer, "%s%s", path, filename);

			processor.SetOutput(buffer, ".jpg");
		}
		break;

		case 1:
		{
			/////////////////////////
			// output: video
			/////////////////////////
			char buffer[100];
			sprintf_s(buffer, "%s%s.mp4", path, filename );

			processor.SetOutput(buffer);
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

	cv::waitKey();
	return 0;
}