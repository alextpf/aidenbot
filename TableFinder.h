#pragma once
#include "LineFinder.h"
#include "Utility.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

//===================================================================================
class TableFinder : public LineFinder
{
public:
	TableFinder(
		METHOD m,
		double dRho,
		double dTheta,
		unsigned int minVote,
		float minLength,
		float maxGap);
	void Refine4Edges(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth );

	void DrawDetectedLines(cv::Mat &image, cv::Scalar color = cv::Scalar(255, 255, 255)) override;

private:

	// refine 4 edges
	void RefineLeftEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth);
	
	void RefineRightEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth);

	void RefineTopEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth);

	void RefineBottomEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth);

	cv::Vec4i FilterLines(
		const std::vector<cv::Point> & corners, 
		unsigned int Xoffset,
		unsigned int Yoffset );
	
	cv::Vec4i m_LeftEdge;
	cv::Vec4i m_RightEdge;
	cv::Vec4i m_TopEdge;
	cv::Vec4i m_BottomEdge;
}; // TableFinder
