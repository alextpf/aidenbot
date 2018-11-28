#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <time.h>

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
		const unsigned int fps,
		const cv::Point& puckPos,
		const cv::Point& bouncePos,
		const cv::Point& predPos,
		const cv::Point& prevPos,
		const cv::Point& botPos,
		const cv::Point& detectedBotPos,
		const cv::Point& puckSpeed,
		const int predictTimeDefence,
		const int predictTimeAttack,
		const unsigned int predictBounceStatus,
        const int botXSpeed,
		const int botYSpeed,
		const int predictStatus,
		const unsigned int botStatus,
		const unsigned int attackStatus,
		const clock_t attackTime,
		const cv::Point2f& avgPuckSpeed ) const;

private:

}; // Logger
