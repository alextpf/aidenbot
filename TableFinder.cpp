#include "TableFinder.h"
#include <opencv2/highgui.hpp>

#define DEBUG

//===================================================================================
TableFinder::TableFinder(
	METHOD m,
	double dRho,
	double dTheta,
	unsigned int minVote,
	float minLength,
	float maxGap)
	:LineFinder(m, dRho, dTheta, minVote, minLength, maxGap)
{}

//===================================================================================
void TableFinder::Refine4Edges(
	const std::vector<cv::Point> & corners,
	unsigned int bandWidth,
    cv::Mat& img /*debug use*/ )
{
	RefineLeftEdge(corners, bandWidth, img );
	RefineRightEdge(corners, bandWidth, img );
	RefineTopEdge(corners, bandWidth, img );
	RefineBottomEdge(corners, bandWidth, img );
}//Refine4Edges

 //===================================================================================
void TableFinder::RefineLeftEdge(
	const std::vector<cv::Point> & corners,
	unsigned int bandWidth,
    cv::Mat& img /*debug use*/ )
{
	unsigned int Xoffset = bandWidth;
	unsigned int Yoffset = bandWidth + 11;

	std::vector<cv::Point> tmpCorners;

	// corners is arranged by: ul, ur, ll, lr
	tmpCorners.push_back( corners[0] );
	tmpCorners.push_back( corners[0] );
	tmpCorners.push_back( corners[2] );
	tmpCorners.push_back( corners[2] );

    cv::Point o_ul, o_ur, o_ll, o_lr; // debug use

	m_LeftEdge = FilterLines(tmpCorners, Xoffset, Yoffset, o_ul, o_ur, o_ll, o_lr );

#ifdef DEBUG
    cv::Scalar color = cv::Scalar( 255, 255, 255 ); // white

    cv::line( img, o_ul, o_ur, color );
    cv::line( img, o_ul, o_ll, color );
    cv::line( img, o_ur, o_lr, color );
    cv::line( img, o_ll, o_lr, color );

    cv::imshow( "line bound:", img );
#endif
}//RefineLeftEdge

 //===================================================================================
void TableFinder::RefineRightEdge(
	const std::vector<cv::Point> & corners,
	unsigned int bandWidth,
    cv::Mat& img /*debug use*/ )
{
	unsigned int Xoffset = bandWidth;
	unsigned int Yoffset = bandWidth + 11;

	// corners is arranged by: ul, ur, ll, lr
	std::vector<cv::Point> tmpCorners;

	tmpCorners.push_back(corners[1]);
	tmpCorners.push_back(corners[1]);
	tmpCorners.push_back(corners[3]);
	tmpCorners.push_back(corners[3]);

    cv::Point o_ul, o_ur, o_ll, o_lr; // debug use

	m_RightEdge = FilterLines(tmpCorners, Xoffset, Yoffset, o_ul, o_ur, o_ll, o_lr );

#ifdef DEBUG
    cv::Scalar color = cv::Scalar( 0, 255, 255 ); // yellow

    cv::line( img, o_ul, o_ur, color );
    cv::line( img, o_ul, o_ll, color );
    cv::line( img, o_ur, o_lr, color );
    cv::line( img, o_ll, o_lr, color );

    cv::imshow( "line bound:", img );
#endif
}//RefineRightEdge

 //===================================================================================
void TableFinder::RefineTopEdge(
	const std::vector<cv::Point> & corners,
	unsigned int bandWidth,
    cv::Mat& img /*debug use*/ )
{
	unsigned int Xoffset = bandWidth + 11;
	unsigned int Yoffset = bandWidth;

	// corners is arranged by: ul, ur, ll, lr
	std::vector<cv::Point> tmpCorners;

	tmpCorners.push_back(corners[0]);
	tmpCorners.push_back(corners[1]);
	tmpCorners.push_back(corners[0]);
	tmpCorners.push_back(corners[1]);

    cv::Point o_ul, o_ur, o_ll, o_lr; // debug use

	m_TopEdge = FilterLines(tmpCorners, Xoffset, Yoffset, o_ul, o_ur, o_ll, o_lr );

#ifdef DEBUG
    cv::Scalar color = cv::Scalar( 0, 0, 255 ); // red

    cv::line( img, o_ul, o_ur, color );
    cv::line( img, o_ul, o_ll, color );
    cv::line( img, o_ur, o_lr, color );
    cv::line( img, o_ll, o_lr, color );

    cv::imshow( "line bound:", img );
#endif
}//RefineTopEdge

 //===================================================================================
void TableFinder::RefineBottomEdge(
	const std::vector<cv::Point> & corners,
	unsigned int bandWidth,
    cv::Mat& img /*debug use*/ )
{
	unsigned int Xoffset = bandWidth + 11;
	unsigned int Yoffset = bandWidth;

	// corners is arranged by: ul, ur, ll, lr
	std::vector<cv::Point> tmpCorners;

	tmpCorners.push_back(corners[2]);
	tmpCorners.push_back(corners[3]);
	tmpCorners.push_back(corners[2]);
	tmpCorners.push_back(corners[3]);

    cv::Point o_ul, o_ur, o_ll, o_lr; // debug use

	m_BottomEdge = FilterLines(tmpCorners, Xoffset, Yoffset, o_ul, o_ur, o_ll, o_lr );

#ifdef DEBUG
    cv::Scalar color = cv::Scalar( 0, 255, 0 ); // green

    cv::line( img, o_ul, o_ur, color );
    cv::line( img, o_ul, o_ll, color );
    cv::line( img, o_ur, o_lr, color );
    cv::line( img, o_ll, o_lr, color );

    cv::imshow( "line bound:", img );
#endif
}//RefineBottomEdge

//===================================================================================
cv::Vec4i TableFinder::FilterLines(
	const std::vector<cv::Point> & corners,
	unsigned int Xoffset,
	unsigned int Yoffset,
    cv::Point& o_ul,
    cv::Point& o_ur,
    cv::Point& o_ll,
    cv::Point& o_lr )
{
	// corners is arranged by: ul, ur, ll, lr
	float o_l, o_r, o_t, o_b; // slope

	Utility::GenerateOuterBand(
		o_l, // slope
		o_r,
		o_t,
		o_b,
		o_ul,
		o_ur,
		o_ll,
		o_lr,
		corners,
		Xoffset,
		Yoffset);

	std::vector<cv::Vec4i> edges;

	std::vector<cv::Vec4i>::const_iterator it = m_LinesP.begin();

	while (it != m_LinesP.end())
	{
		cv::Point pt1((*it)[0], (*it)[1]);
		cv::Point pt2((*it)[2], (*it)[3]);

		// if both ends of the line are inside the band, the line survives
		if ( !Utility::IsOutsideOuter( pt1.x, pt1.y, o_l, o_r, o_t, o_b, o_ur, o_ll, o_lr) &&
			 !Utility::IsOutsideOuter( pt2.x, pt2.y, o_l, o_r, o_t, o_b, o_ur, o_ll, o_lr) )
		{
			edges.push_back(*it);
		}
		++it;
	}

	if (edges.size() > 1)
	{
		// pick the line that has the longest length
		float maxSqrLen = 0.0f;
		cv::Vec4i tmp;
		for (int i = 0; i < edges.size(); i++)
		{
			cv::Point pt1(edges[i][0], edges[i][1]);
			cv::Point pt2(edges[i][2], edges[i][3]);
			float sqrLen = (pt1.x - pt2.x) * (pt1.x - pt2.x) +
				(pt1.y - pt2.y) * (pt1.y - pt2.y);
			if (sqrLen > maxSqrLen)
			{
				tmp = edges[i];
                maxSqrLen = sqrLen;
			}
		}

		return tmp;
	}

	return edges[0];
}// FilterLines

//===================================================================================
void TableFinder::DrawTableLines(cv::Mat &image, cv::Scalar color)
{
	int thickness = 2;
	if (m_Method == PROB)
	{
		// Draw the lines

		cv::Point pt1, pt2;

		// left
		pt1 = cv::Point( m_LeftEdge[0], m_LeftEdge[1] );
		pt2 = cv::Point( m_LeftEdge[2], m_LeftEdge[3] );

		cv::line(image, pt1, pt2, color, thickness);

		// right
		pt1 = cv::Point(m_RightEdge[0], m_RightEdge[1]);
		pt2 = cv::Point(m_RightEdge[2], m_RightEdge[3]);

		cv::line(image, pt1, pt2, color, thickness);

		// top
		pt1 = cv::Point(m_TopEdge[0], m_TopEdge[1]);
		pt2 = cv::Point(m_TopEdge[2], m_TopEdge[3]);

		cv::line(image, pt1, pt2, color, thickness);

		// bottom
		pt1 = cv::Point(m_BottomEdge[0], m_BottomEdge[1]);
		pt2 = cv::Point(m_BottomEdge[2], m_BottomEdge[3]);

		cv::line(image, pt1, pt2, color, thickness);
	}
	else
	{
		DrawDetectedLines( image, color );
	}
}//DrawDetectedLines