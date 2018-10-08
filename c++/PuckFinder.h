#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

typedef std::vector<std::vector<cv::Point> > Contours;

class PuckFinder 
{
public:
	//============================================
	// @param [out] contours of size 1
	// @param [out] pt: center of contour
	// @param [in] input : input image
	// @param [in] mask: input mask
	bool FindPuck( 
		Contours& contours, 
		cv::Point& pt,
		const cv::Mat& input,
		const cv::Mat& mask );
}; // PuckFinder
