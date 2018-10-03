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
		unsigned int bandWidth,
        cv::Mat& img /*debug use*/);

	void DrawTableLines(cv::Mat &image, cv::Scalar color = cv::Scalar(255, 255, 255));

private:

	// refine 4 edges
	void RefineLeftEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth,
        cv::Mat& img /*debug use*/ );

	void RefineRightEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth,
        cv::Mat& img /*debug use*/ );

	void RefineTopEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth,
        cv::Mat& img /*debug use*/ );

	void RefineBottomEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth,
        cv::Mat& img /*debug use*/ );

	cv::Vec4i FilterLines(
		const std::vector<cv::Point> & corners,
		unsigned int Xoffset,
		unsigned int Yoffset,
        cv::Point& o_ul, // debug use
        cv::Point& o_ur, // debug use
        cv::Point& o_ll, // debug use
        cv::Point& o_lr ); // debug use

	cv::Vec4i m_LeftEdge;
	cv::Vec4i m_RightEdge;
	cv::Vec4i m_TopEdge;
	cv::Vec4i m_BottomEdge;
}; // TableFinder
