#pragma once
#include "LineFinder.h"
#include "Utility.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#define TABLE_LENGTH    1003  // mm = 39.5''
#define TABLE_WIDTH     597   // mm = 23.5''
//===================================================================================
class TableFinder : public LineFinder
{
public:
	TableFinder() :LineFinder()
	{}

	TableFinder(
		METHOD m,
		double dRho,
		double dTheta,
		unsigned int minVote,
		float minLength,
		float maxGap);

	bool Refine4Edges(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth,
        cv::Mat& img /*debug use*/);

	void DrawTableLines(cv::Mat &image, cv::Scalar color = cv::Scalar(255, 255, 255));

	const cv::Point& GetTopLeft()
	{
		return m_TopLeft;
	}

	const cv::Point& GetTopRight()
	{
		return m_TopRight;
	}

	const cv::Point& GetLowerLeft()
	{
		return m_LowerLeft;
	}

	const cv::Point& GetLowerRight()
	{
		return m_LowerRight;
	}

	void SetTopLeft( const cv::Point& p )
	{
		m_TopLeft = p;
	}

	void SetTopRight( const cv::Point& p )
	{
		m_TopRight = p;
	}

	void SetLowerLeft( const cv::Point& p )
	{
		m_LowerLeft = p;
	}

	void SetLowerRight( const cv::Point& p )
	{
		m_LowerRight = p;
	}

	// assuming robot is on the left of the screen
	//
	//  Robot Coordinate:
	//
	//
	//  ---------------------------->Y
	//  |                       |
	//  |                       |
	//  |                       |
	//  |------------------------
	//  |  X
	//  V

	//  Image Coordinate:
	//  --------------------------->X
	//  |                       |
	//  |                       |
	//  |                       |
	//  |------------------------
	//  |
	//  V Y
	cv::Point ImgToTableCoordinate( cv::Point p);

	cv::Point TableToImgCoordinate( cv::Point p );

	float GetLeft()
	{
		return m_Left;
	}

	float GetRight()
	{
		return m_Right;
	}

	float GetTop()
	{
		return m_Top;
	}

	float GetBottom()
	{
		return m_Bottom;
	}
private:

	// refine 4 edges
	bool RefineLeftEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth,
        cv::Mat& img /*debug use*/ );

	bool RefineRightEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth,
        cv::Mat& img /*debug use*/ );

	bool RefineTopEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth,
        cv::Mat& img /*debug use*/ );

	bool RefineBottomEdge(
		const std::vector<cv::Point> & corners,
		unsigned int bandWidth,
        cv::Mat& img /*debug use*/ );

	bool FilterLines(
		cv::Vec4i& edge,
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

	// 4 corners
	cv::Point m_TopLeft;
	cv::Point m_TopRight;
	cv::Point m_LowerLeft;
	cv::Point m_LowerRight;

	// This is an oversimplified model that assumes
	// image plane is parellel to table with no tilt
	// and ignore any lense distortion.
	// So the resulting table boundary is a rectangle on screen
	float m_Left;
	float m_Right;
	float m_Top;
	float m_Bottom;

	// camera pixel to table mm
	float m_PixToMM;

}; // TableFinder
