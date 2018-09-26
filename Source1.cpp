
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

#include "C:\Users\alex_\Documents\Arduino\aidenbot\v2\aidenbot\VideoProcessor.h"

using namespace cv;
using namespace std;

int main()
{
	// Create video procesor instance
	VideoProcessor processor;

	//////////////////
	// variables
	//////////////////

	char path[] = "C:/tmp/";
	char filename[] = "test1";
	int num = 91;		
	int startFrame = 1;// frame number we want to start at
	char inputMode[] = "";
	char outputMode[] = "";

	/////////////////////
	// Input & Output
	/////////////////////
	// 0: imgs, 1: video, 2: webcam
	static int inputType = 0;
	int webCamId = 0; // 0: default

	// 0: imgs, 1: video, 2: no output written
	static int outputType = 1;

	/////////////////
	// Frame Rate:
	/////////////////
	float fps = 60.f;

	float delay = 1000. / fps;

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

	// Declare a window to display the video
	processor.DisplayOutput("Test Output");

	// Declare a window to display the input
	processor.DisplayInput("Input");

	processor.SetFrameNumber(startFrame);
	processor.SetDelay(delay);

	// Start the Process
	processor.Run();

	return 0;
}