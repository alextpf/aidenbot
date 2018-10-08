#include "LineFinder.h"
#include "Utility.h"

//===================================================================================
// Default accumulator resolution is 1 pixel by 1 degree
// no gap, no mimimum length
LineFinder::LineFinder()
	: m_Method( TRAD )
	, m_DeltaRho( 1.0f )
	, m_DeltaTheta( CV_PI / 180.0f )
	, m_MinVote( 10 )
	, m_MinLength( 0.f )
	, m_MaxGap( 0.f )
{}

//===================================================================================
LineFinder::LineFinder(
	METHOD method,
	double dRho,
	double dTheta,
	unsigned int minVote,
	float minLength,
	float maxGap)
	: m_Method( method )
	, m_DeltaRho( dRho )
	, m_DeltaTheta( dTheta )
	, m_MinVote( minVote )
	, m_MinLength( minLength )
	, m_MaxGap( maxGap )
{}

//===================================================================================
LineFinder::~LineFinder()
{}

//===================================================================================
const std::vector<cv::Vec4i>& LineFinder::FindLinesP(cv::Mat& binary)
{
	if (m_Method != PROB)
	{
		return m_LinesP;
	}

	m_LinesP.clear();

	cv::HoughLinesP(
		binary,
		m_LinesP,
		m_DeltaRho,
		m_DeltaTheta,
		m_MinVote,
		m_MinLength,
		m_MaxGap);

	return m_LinesP;
}//FindLinesP

//===================================================================================
const std::vector<cv::Vec2f>& LineFinder::FindLines(cv::Mat& binary)
{
	if (m_Method != TRAD)
	{
		return m_Lines;
	}

	m_Lines.clear();

	cv::HoughLines(
		binary,
		m_Lines,
		m_DeltaRho,
		m_DeltaTheta,
		m_MinVote);

	return m_Lines;
}//FindLines

 //===================================================================================
void LineFinder::FilterDetectedLines(
    const std::vector<cv::Point> & corners,
    unsigned int bandWidth,
    cv::Size s)
{
    if( m_Method != PROB )
    {
        return;
    }

    float o_l, o_r, o_t, o_b;
    float i_l, i_r, i_t, i_b;

    // outer 4 corners
    cv::Point o_ul;
    cv::Point o_ur;
    cv::Point o_ll;
    cv::Point o_lr;

    // inner 4 corners
    cv::Point i_ul;
    cv::Point i_ur;
    cv::Point i_ll;
    cv::Point i_lr;

    Utility::GenerateBand(
        o_l, o_r, o_t, o_b,
        i_l, i_r, i_t, i_b,
        o_ul, o_ur, o_ll, o_lr,
        i_ul, i_ur, i_ll, i_lr,
        corners, bandWidth, bandWidth);

    std::vector<cv::Vec4i> newLines;

    std::vector<cv::Vec4i>::const_iterator it = m_LinesP.begin();

    while( it != m_LinesP.end() )
    {
        cv::Point pt1( ( *it )[0], ( *it )[1] );
        cv::Point pt2( ( *it )[2], ( *it )[3] );

        // if both ends of the line are inside the band, the line survives
        if( !Utility::IsOutsideOuter( pt1.x, pt1.y, o_l, o_r, o_t, o_b, o_ur, o_ll, o_lr ) &&
            !Utility::IsInsideInner ( pt1.x, pt1.y, i_l, i_r, i_t, i_b, i_ur, i_ll, i_lr ) &&
            !Utility::IsOutsideOuter( pt2.x, pt2.y, o_l, o_r, o_t, o_b, o_ur, o_ll, o_lr ) &&
            !Utility::IsInsideInner ( pt2.x, pt2.y, i_l, i_r, i_t, i_b, i_ur, i_ll, i_lr ) )
        {
            newLines.push_back( *it );
        }
        ++it;
    }

    m_LinesP = newLines;

} // FilterDetectedLines

 //===================================================================================
void LineFinder::DrawDetectedLines(cv::Mat &image, cv::Scalar color )
{
	int thickness = 2;
	if (m_Method == PROB)
	{
		// Draw the lines
		std::vector<cv::Vec4i>::const_iterator it = m_LinesP.begin();

		while (it != m_LinesP.end())
		{
			cv::Point pt1((*it)[0], (*it)[1]);
			cv::Point pt2((*it)[2], (*it)[3]);

			cv::line(image, pt1, pt2, color, thickness);

			++it;
		}
	}
	else
	{
		// TRAD method
		for (size_t i = 0; i < m_Lines.size(); i++)
		{
			float rho = m_Lines[i][0];
			float theta = m_Lines[i][1];
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			cv::Point pt1(
				cvRound(x0 + 1000 * (-b)),
				cvRound(y0 + 1000 * (a)));
			cv::Point pt2(
				cvRound(x0 - 1000 * (-b)),
				cvRound(y0 - 1000 * (a)));

			cv::line(image, pt1, pt2, color, thickness);
		}
	}
}//DrawDetectedLines