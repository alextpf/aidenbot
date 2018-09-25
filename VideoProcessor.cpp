#include "VideoProcessor.h"

//=======================================================================
VideoProcessor::VideoProcessor() :
callIt(false),
delay(-1),
fnumber(0),
totalFrame(0),
stop(false),
digits(0),
frameToStop(-1),
Process(0),
frameProcessor(0),
initPosX(-1),
initPosY(-1),
offsetX(-1),
offsetY(-1)
{}

//=======================================================================
bool VideoProcessor::readNextFrame(cv::Mat& frame)
{
	bool ok = false;

	if (images.size() == 0)
	{
		////////////////
		// it's video
		////////////////
		ok = capture.read(tmpFrame);
	}
	else
	{
		////////////////
		// it's images
		////////////////
		if (itImg != images.end())
		{
			printf("%s\n", (*itImg).c_str());
			tmpFrame = cv::imread(*itImg);
			itImg++;

			ok = tmpFrame.data != 0;
		}
	}

	// whether we extract only portion of the image
	if (offsetX > 0 && offsetY > 0 && initPosX >= 0 && initPosY >= 0)
	{
		frame = tmpFrame(cv::Rect(initPosX, initPosY, offsetX, offsetY)).clone();
	}
	else
	{
		frame = tmpFrame;
	}

	return ok;
}

//=======================================================================
void VideoProcessor::writeNextFrame(cv::Mat& frame)
{
	if (extension.length())
	{
		////////////////
		// it's images
		////////////////

		std::stringstream ss;
		ss << outputFile << std::setfill('0') << std::setw(digits) << currentIndex++ << extension;
		cv::imwrite(ss.str(), frame);

	}
	else
	{
		////////////////
		// it's video
		////////////////
		writer.write(frame);
	}
}

//=======================================================================      
bool VideoProcessor::SetInput(std::string filename)
{
	fnumber = 0;
	totalFrame = 0;
	// In case a resource was already
	// associated with the VideoCapture instance
	capture.release();
	images.clear();

	// Open the video file
	return capture.open(filename);
}

//=======================================================================
bool VideoProcessor::SetInput(int id)
{
	fnumber = 0;
	totalFrame = 0;
	// In case a resource was already
	// associated with the VideoCapture instance
	capture.release();
	images.clear();

	// Open the video file
	return capture.open(id);
}

//=======================================================================
bool VideoProcessor::SetInput(const std::vector<std::string>& imgs)
{
	fnumber = 0;
	totalFrame = 0;
	// In case a resource was already
	// associated with the VideoCapture instance
	capture.release();

	// the input will be this vector of images
	images = imgs;
	itImg = images.begin();

	return true;
}

//=======================================================================
bool VideoProcessor::setOutput(
	const std::string &filename, 
	int codec,
	double framerate,
	bool isColor)
{
	outputFile = filename;
	extension.clear();

	if (framerate == 0.0)
	{
		framerate = getFrameRate(); // same as input
	}

	char c[4];
	// use same codec as input
	if (codec == 0)
	{
		codec = getCodec(c);
	}

	// Open output video
	return writer.open(outputFile, // filename
		codec, // codec to be used
		framerate,      // frame rate of the video
		getFrameSize(), // frame size
		isColor);       // color video?
}

//=======================================================================
bool VideoProcessor::setOutput(
	const std::string &filename, // filename prefix
	const std::string &ext, // image file extension
	int numberOfDigits,   // number of digits
	int startIndex)
{     

	// number of digits must be positive
	if (numberOfDigits < 0)
		return false;

	// filenames and their common extension
	outputFile = filename;
	extension = ext;

	// number of digits in the file numbering scheme
	digits = numberOfDigits;
	// start numbering at this index
	currentIndex = startIndex;

	return true;
}

//=======================================================================
void VideoProcessor::setFrameProcessor(void(*frameProcessingCallback)(cv::Mat&, cv::Mat&)) 
{
	// invalidate frame processor class instance
	frameProcessor = 0;
	// this is the frame processor function that will be called
	Process = frameProcessingCallback;
	callProcess();
}

//=======================================================================
void VideoProcessor::setFrameProcessor(FrameProcessor* frameProcessorPtr) 
{
	// invalidate callback function
	Process = 0;
	// this is the frame processor instance that will be called
	frameProcessor = frameProcessorPtr;
	callProcess();
}

//=======================================================================
cv::Size VideoProcessor::getFrameSize() 
{
	if (images.size() == 0) {

		// get size of from the capture device
		int w = static_cast<int>(capture.get(CV_CAP_PROP_FRAME_WIDTH));
		int h = static_cast<int>(capture.get(CV_CAP_PROP_FRAME_HEIGHT));

		return cv::Size(w, h);

	}
	else { // if input is vector of images

		cv::Mat tmp = cv::imread(images[0]);
		if (!tmp.data) return cv::Size(0, 0);
		else return tmp.size();
	}
}

//=======================================================================
long VideoProcessor::getFrameNumber()
{
	if (images.size() == 0) {

		// get info of from the capture device
		long f = static_cast<long>(capture.get(CV_CAP_PROP_POS_FRAMES));
		return f;

	}
	else { // if input is vector of images

		return static_cast<long>(itImg - images.begin());
	}
}

//=======================================================================
double VideoProcessor::getPositionMS()
{
	// undefined for vector of images
	if (images.size() != 0) return 0.0;

	double t = capture.get(CV_CAP_PROP_POS_MSEC);
	return t;
}

//=======================================================================
double VideoProcessor::getFrameRate() 
{
	// undefined for vector of images
	if (images.size() != 0) return 0;

	double r = capture.get(CV_CAP_PROP_FPS);
	return r;
}

//=======================================================================
long VideoProcessor::getTotalFrameCount()
{
	// for vector of images
	if (images.size() != 0) return images.size();

	long t = capture.get(CV_CAP_PROP_FRAME_COUNT);
	return t;
}

//=======================================================================
int VideoProcessor::getCodec(char codec[4])
{
	// undefined for vector of images
	if (images.size() != 0) return -1;

	union {
		int value;
		char code[4];
	} returned;

	returned.value = static_cast<int>(capture.get(CV_CAP_PROP_FOURCC));

	codec[0] = returned.code[0];
	codec[1] = returned.code[1];
	codec[2] = returned.code[2];
	codec[3] = returned.code[3];

	return returned.value;
}

//=======================================================================
bool VideoProcessor::setFrameNumber(long pos)
{
	// for vector of images
	if (images.size() != 0)
	{
		// move to position in vector
		itImg = images.begin() + pos;

		// is it a valid position?
		return (pos < images.size());
	}
	else
	{
		// if input is a capture device
		return capture.set(CV_CAP_PROP_POS_FRAMES, pos);
	}
}

//=======================================================================
bool VideoProcessor::setPositionMS(double pos)
{
	// not defined in vector of images
	if (images.size() != 0)
		return false;
	else
		return capture.set(CV_CAP_PROP_POS_MSEC, pos);
}

//=======================================================================
bool VideoProcessor::setRelativePosition(double pos)
{
	// for vector of images
	if (images.size() != 0) {

		// move to position in vector
		long posI = static_cast<long>(pos*images.size() + 0.5);
		itImg = images.begin() + posI;
		// is it a valid position?
		if (posI < images.size())
			return true;
		else
			return false;

	}
	else { // if input is a capture device

		return capture.set(CV_CAP_PROP_POS_AVI_RATIO, pos);
	}
}


//=======================================================================
void VideoProcessor::run()
{
	/*
	sf::SoundBuffer buffer;
	if (!buffer.loadFromFile("test2.wav"))
	{
	return;
	}
	sf::Sound sound;
	sound.setBuffer(buffer);
	*/
	/*
	sound.stop();
	sound.setBuffer(buffer);
	sound.play();*/

	// current frame
	cv::Mat frame;
	// output frame
	cv::Mat output;

	// if no capture device has been set
	if (!isOpened())
	{
		return;
	}

	stop = false;

	while (!isStopped())
	{
		// read next frame if any
		if (!readNextFrame(frame))
		{
			break;
		}

		// display input frame
		if (windowNameInput.length() != 0)
		{
			cv::imshow(windowNameInput, frame);
		}

		// calling the Process function or method
		if (callIt)
		{
			// Process the frame
			if (Process)
			{
				Process(frame, output);
			}
			else if (frameProcessor)
			{
				frameProcessor->Process(frame, output);
			}
			// increment frame number
			fnumber++;	
			std::cout << fnumber << std::endl;//print the number for debug
		}
		else
		{
			output = frame;
		}
		
		totalFrame++;
		std::cout << totalFrame << std::endl;//print the number for debug

		// write output sequence
		if (outputFile.length() != 0)
		{
			writeNextFrame(output);
		}

		// display output frame
		if (windowNameOutput.length() != 0)
		{
			cv::imshow(windowNameOutput, output);
		}

		//double a = cv::sum (output)[0];
		//
		//if ( a > 5000000.0 )
		//{
		// //std::cout << '\a';
		// sf::Sound::Status s = sound.getStatus();

		// if ( s != sf::Sound::Status::Playing )
		// {
		//  // sound.setBuffer(buffer);
		//  sound.play();
		// }
		//}

		// introduce a delay
		if (delay >= 0 )
		{
			int ret = cv::waitKey(delay);
			if (ret > 0)
			{
				//stopIt();
			}
		}
		else
		{
			cv::waitKey(delay);
		}

		// check if we should stop
		if (frameToStop >= 0 && getFrameNumber() == frameToStop)
		{
			stopIt();
		}
	}
	/*
	sf::SoundBuffer buffer;
	if (!buffer.loadFromFile("test1.wav"))
	{
	return;
	}
	sf::Sound sound;
	sound.stop();
	sound.setBuffer(buffer);
	sound.play();*/
}