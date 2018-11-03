#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class Logger
{
public:

	// ctor
	Logger();

	void WriteTableCorners(
		cv::Point& TopLeft,
		cv::Point& TopRight,
		cv::Point& LowerLeft,
		cv::Point& LowerRight );

	void LogStatus( long numFrame, unsigned int dt, unsigned int fps );
}; // Logger
