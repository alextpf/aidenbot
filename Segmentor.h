#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

#include "videoprocessor.h"
#include <list>

class Segmentor : public FrameProcessor
{
public:
	Segmentor();
	~Segmentor() {}

	virtual void Process(cv::Mat & input, cv::Mat & output);	
	void SetXOutterOffset(unsigned int x);
	void SetYOutterOffset(unsigned int y);
	void SetXInnerOffset(unsigned int x);
	void SetYInnerOffset(unsigned int y);

	static void OnMouse(int event, int x, int y, int f, void* data);

private:
	bool m_TableFound;
	
	// user picked 4 corners of table
	std::vector<cv::Point> m_Corners;

	unsigned int m_OuterXOffset;
	unsigned int m_OuterYOffset;
	unsigned int m_InnerXOffset;
	unsigned int m_InnerYOffset;
};