#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class Logger
{
public:

	// ctor
	Logger();

	void WriteTableCorners(
		const cv::Point& TopLeft,
		const cv::Point& TopRight,
		const cv::Point& LowerLeft,
		const cv::Point& LowerRight ) const;

	void LogStatus(
		const long numFrame,
		const unsigned int dt,
		const unsigned int fps ) const;

	void LogStatus(
		const long numFrame,
		const unsigned int dt,
		const unsigned int fps,
		const cv::Point& puckPos,
		const cv::Point& bouncePos,
		const cv::Point& predPos,
		const cv::Point& prevPos,
		const cv::Point& botPos,
		const int botSpeed,
		const int predictStatus,
		const unsigned int botStatus ) const;

private:

}; // Logger
