#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

#include "videoprocessor.h"

class ImgComposer : public FrameProcessor
{
public:
	//bool ReadLog();
	void Process( cv::Mat & input, cv::Mat & output ) override;
private:
	std::vector<cv::Point>	m_Corners;
};//ImgComposer
