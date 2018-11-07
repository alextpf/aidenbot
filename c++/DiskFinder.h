#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

typedef std::vector<std::vector<cv::Point> > Contours;

class DiskFinder
{
public:
	//============================================
	// @param [out] contours of puck of size 1
	// @param [out] puckCenter : center of puck contour
	// @param [in] hsvImg : input HSV image
	// @param [in] mask: input mask
	bool FindDisk1Thresh(
		Contours& contours,
		cv::Point& center,
		const cv::Mat& hsvImg,		//HSV imag
		const cv::Vec6i& thresh,
		const cv::Mat& mask = cv::Mat() );

	bool FindDisk2Thresh(
		Contours& contours,
		cv::Point& center,
		const cv::Mat& hsvImg,
		const cv::Vec6i& thresh1,
		const cv::Vec6i& thresh2,
		const cv::Mat& mask = cv::Mat() );

	void SetAreaLow( const double low )
	{
		m_AreaLow = low;
	}

	void SetAreaHigh( const double high )
	{
		m_AreaHigh = high;
	}

private:

	bool FindDiskInternal(
		Contours& contours,
		cv::Point& center,
		const cv::Mat& mask = cv::Mat() );

	double	m_AreaLow;
	double	m_AreaHigh;
}; // DiskFinder
