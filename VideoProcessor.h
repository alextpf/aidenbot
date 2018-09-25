
#ifndef VPROCESSOR
#define VPROCESSOR

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// The frame processor interface
class FrameProcessor 
{

public:
	// processing method
	virtual void Process(cv::Mat &input, cv::Mat &output) = 0;
};

class VideoProcessor 
{
public:

	// Constructor setting the default values
	VideoProcessor();

	// set the name of the video file
	bool SetInput(std::string filename);

	// set the camera ID
	bool SetInput(in0t id);

	// set the vector of input images
	bool SetInput(const std::vector<std::string>& imgs);

	// set the output video file
	// by default the same parameters than input video will be used
	bool SetOutput(
		const std::string& filename,
		int codec = 0,
		double framerate = 0.0,
		bool isColor = true);

	// set the output as a series of image files
	// extension must be ".jpg", ".bmp" ...
	bool SetOutput(
		const std::string& filename, // filename prefix
		const std::string& ext, // image file extension
		int numberOfDigits = 3,   // number of digits
		int startIndex = 0);

	// set the callback function that will be called for each frame
	void SetFrameProcessor( void( *frameProcessingCallback ) ( cv::Mat&, cv::Mat& ) );

	// set the instance of the class that implements the FrameProcessor interface
	void SetFrameProcessor(FrameProcessor* frameProcessorPtr);

	// stop streaming at this frame number
	void StopAtFrameNo(long frame) {

		frameToStop = frame;
	}

	// Process callback to be called
	void CallProcess() 
	{
		callIt = true;
	}

	// do not call Process callback
	void DontCallProcess() 
	{
		callIt = false;
	}

	// to display the processed frames
	void DisplayInput(std::string wn) 
	{
		windowNameInput = wn;
		cv::namedWindow(windowNameInput);
	}

	// to display the processed frames
	void DisplayOutput(std::string wn)
	{
		windowNameOutput = wn;
		cv::namedWindow(windowNameOutput);
	}

	// do not display the processed frames
	void DontDisplay()
	{
		cv::destroyWindow(windowNameInput);
		cv::destroyWindow(windowNameOutput);
		windowNameInput.clear();
		windowNameOutput.clear();
	}

	// set a delay between each frame
	// 0 means wait at each frame
	// negative means no delay
	void SetDelay(int d) 
	{
		delay = d;
	}

	// a count is kept of the processed frames
	long GetNumberOfProcessedFrames() 
	{
		return fnumber;
	}

	// return the size of the video frame
	cv::Size GetFrameSize();

	// return the frame number of the next frame
	long GetFrameNumber();

	// return the position in ms
	double GetPositionMS();

	// return the frame rate
	double GetFrameRate();

	// return the number of frames in video
	long GetTotalFrameCount();

	// get the codec of input video
	int GetCodec(char codec[4]);

	// go to this frame number
	bool GetFrameNumber(long pos);

	// go to this position
	bool GetPositionMS(double pos);

	// go to this position expressed in fraction of total film length
	bool GetRelativePosition(double pos);

	// Stop the processing
	void GtopIt()
	{
		stop = true;
	}

	// Is the Process stopped?
	bool GsStopped()
	{
		return stop;
	}

	// Is a capture device opened?
	bool GsOpened()
	{
		return capture.isOpened() || !images.empty();
	}

	void SetInitPosX(int x)
	{
		initPosX = x;
	}

	void SetInitPosY(int y)
	{
		initPosY = y;
	}

	void SetOffsetX(int x)
	{
		offsetX = x;
	}

	void SetOffsetY(int y)
	{
		offsetY = y;
	}

	// to grab (and Process) the frames of the sequence
	void Run();

private:

	// the OpenCV video capture object
	cv::VideoCapture m_Capture;

	// the callback function to be called
	// for the processing of each frame
	void (*m_Process) ( cv::Mat&, cv::Mat& );
	
	// the pointer to the class implementing
	// the FrameProcessor interface
	FrameProcessor *m_FrameProcessor;

	// a bool to determine if the
	// Process callback will be called
	bool m_CallIt;
	// Input display window name
	std::string windowNameInput;
	// Output display window name
	std::string windowNameOutput;
	// delay between each frame processing
	int delay;
	// number of processed frames
	long fnumber;

	long totalFrame;

	// stop at this frame number
	long frameToStop;
	// to stop the processing
	bool stop;

	// vector of image filename to be used as input
	std::vector<std::string> images;
	// image vector iterator
	std::vector<std::string>::const_iterator itImg;

	// the OpenCV video writer object
	cv::VideoWriter writer;
	// output filename
	std::string outputFile;

	// current index for output images
	int currentIndex;
	// number of digits in output image filename
	int digits;
	// extension of output images
	std::string extension;

	int initPosX;
	int initPosY;
	int offsetX; //
	int offsetY; //
	cv::Mat tmpFrame; // tmp frame

					  // to get the next frame
					  // could be: video file; camera; vector of images
	bool readNextFrame(cv::Mat& frame);

	// to write the output frame
	// could be: video file or images
	void writeNextFrame(cv::Mat& frame);

};

#endif
