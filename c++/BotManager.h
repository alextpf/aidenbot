#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <fstream>
#include <list>

#include "TableFinder.h"
#include "videoprocessor.h"
#include "Camera.h"
#include "Robot.h"
#include "DiskFinder.h"
#include "SerialPort.h"
#include "FPSCalculator.h"
#include "Logger.h"

class BotManager : public FrameProcessor
{
public:
	BotManager( char* com );
	~BotManager() {}

	void Process(cv::Mat & input, cv::Mat & output) override;

	static void OnMouse(int event, int x, int y, int f, void* data);

	void SetBandWidth(unsigned int w)
	{
		m_BandWidth = w;
	}

    unsigned int GetBandWidth()
    {
        return m_BandWidth;
    }

	void SetShowDebugImg( const bool ok );

	void SetManualPickTableCorners( const bool ok );

	void SetShowOutPutImg( const bool ok );

	bool IsSerialConnected()
	{
		return m_pSerialPort->IsConnected();
	}

	void SetRedThreshold( const cv::Vec6i& red )
	{
		m_RedThresh = red;
	}

	void SetOrangeThreshold( const cv::Vec6i& orange )
	{
		m_OrangeThresh = orange;
	}

	void SetBlueThreshold( const cv::Vec6i& blue )
	{
		m_BlueThresh = blue;
	}

	void SetIsLog( const bool isLog )
	{
		m_IsLog = isLog;
	}

	void SetPuckAreaThreshLow( const double low )
	{
		m_PuckFinder.SetAreaLow( low );
	}

	void SetPuckAreaThreshHigh( const double high )
	{
		m_PuckFinder.SetAreaHigh( high );
	}

	void SetBotAreaThreshLow( const double low )
	{
		m_BotFinder.SetAreaLow( low );
	}

	void SetBotAreaThreshHigh( const double high )
	{
		m_BotFinder.SetAreaHigh( high );
	}

	void SetCorrectMissingSteps( const bool ok )
	{
		m_CorrectMissingSteps = ok;
	}

    bool CorrectMissingSteps( const bool botFound );

private:

	// wrapper function to find table corners
	void FindTable( cv::Mat & input );

	// send the message to Arduino over com port
	bool SendBotMessage( const bool correctSteps );

    // send the message to Arduino over com port
    void ReceiveMessage(); // debug tuse

	// find ul, ur, ll, lr corners of user input
	void OrderCorners();

	// use the user-picked band to zero out the result of Canny
	void MaskCanny(cv::Mat & img);

	// find robot pos
	bool FindRobot(
		cv::Point& detectedBotPos,
		const cv::Mat& hsvImg,
		cv::Mat & output,
		const unsigned int dt );

	// find puck
	bool FindPuck(
		cv::Point& detectedPuckPos,
		const cv::Mat& hsvImg,
		cv::Mat & output,
		const unsigned int dt,
		const unsigned int fps,
		bool& ownGoal,
		cv::Point& prevPuckPos,
		cv::Point& predPuckPos,
		cv::Point& bouncePos,
		cv::Point& desiredBotPos );

	void TestMotion();

	TableFinder m_TableFinder;
    bool m_TableFound;

	// user picked 4 corners of table
	std::vector<cv::Point> m_Corners;

	// offset around the quadrilateral
	unsigned int m_BandWidth;

	// outer 4 corners
	cv::Point m_o_ul;
	cv::Point m_o_ur;
	cv::Point m_o_ll;
	cv::Point m_o_lr;

	// inner 4 corners
	cv::Point m_i_ul;
	cv::Point m_i_ur;
	cv::Point m_i_ll;
	cv::Point m_i_lr;

    cv::Mat			m_Mask;     // mask represents table area

	clock_t			m_CurrTime;
	Camera			m_Camera;
	Robot			m_Robot;
	std::shared_ptr<SerialPort>		m_pSerialPort;
	bool			m_ShowDebugImg;
	bool			m_ShowOutPutImg;
	bool			m_ManualPickTableCorners;
	long			m_NumFrame;
	cv::Vec6i		m_RedThresh; // for puck
	cv::Vec6i		m_OrangeThresh; // for puck
	cv::Vec6i		m_BlueThresh; // for robot
	bool			m_IsLog;
	unsigned int	m_NumConsecutiveNonPuck; // number of consecutive frames that no puck is detected
	DiskFinder		m_PuckFinder;
	DiskFinder		m_BotFinder;
	FPSCalculator	m_FpsCalculator;
	Logger			m_Logger;
	bool			m_CorrectMissingSteps;
};