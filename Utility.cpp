#include "Utility.h"

//=======================================================================
float Slope( const cv::Point& p1, const cv::Point& p2 )
{
	float s;
	int dx = p2.x - p1.x;
	int dy = p2.y - p1.y;
	if ( dx == 0 )
	{
		s = 10000;//large number
	}
	else
	{
		s = dy / dx;
	}
	return s;
}//Slope

//=======================================================================
bool Utility::FindLineIntersection(
	const cv::Vec4i& l1,
	const cv::Vec4i& l2,
	cv::Point& intr,
	float& s1,
	float& s2 )
{
	// 2 end points of line1
	// p2 has to be on the right of p1
	cv::Point p1( l1[0], l1[1] );
	cv::Point p2( l1[2], l1[3] );
	if ( p1.x > p2.x )
	{
		std::swap( p1, p2 );
	}

	// 2 end points of line2
	// p4 has to be on the right of p3
	cv::Point p3( l2[0], l2[1] );
	cv::Point p4( l2[2], l2[3] );
	if ( p3.x > p4.x )
	{
		std::swap( p3, p4 );
	}

	// note: line equation: y = mx + t
	// find the slope
	float m1 = Slope( p1, p2 );
	float m2 = Slope( p3, p4 );

	if ( m1 == m2 )
	{
		return false;
	}

	// find the intersection
	float t1 = p1.y - m1 * p1.x;
	float t2 = p3.y - m2 * p3.x;

	if ( m1 >= 10000 ) // l1 vertical line
	{
		intr.x = p1.x;
		intr.y = m2 * p1.x + t2;

		return true;
	}

	if ( m2 >= 10000 ) // l2 vertical line
	{
		intr.x = p3.x;
		intr.y = m1 * p3.x + t1;

		return true;
	}

	intr.x = ( t2 - t1 ) / ( m1 - m2 );
	intr.y = ( m1 * t2 + m2 * t1 ) / ( m1 - m2 );

	return true;
	
}//FindLineIntersection

//=======================================================================
// Specialized version of computing slope function
// p2 is the origin, X postive right-ward, Y positive going up-ward
float GetSlope( const cv::Point& p1, const cv::Point& p2 )
{
    int dx = p1.x - p2.x;
    int dy = p2.y - p1.y; // reversed y

    if( dx == 0 )
    {
        return 100000.0f;
    }
    else
    {
        return (float)dy / (float)dx; // line equation: y = x * slope
    }
} // GetSlope
//=======================================================================
// p2 is the origin, X postive left-ward, Y positive going up-ward
float GetInvSlope( const cv::Point& p1, const cv::Point& p2 )
{
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y; // reversed y

    if( dy == 0 )
    {
        return 100000.0f;
    }
    else
    {
        return (float)dx / (float)dy; // line equation: x = y * slope
    }
} // GetInvSlope

//=======================================================================
void Utility::GenerateOuterBand(
	float& o_l, // slope
	float& o_r,
	float& o_t,
	float& o_b,
	cv::Point& o_ul,
	cv::Point& o_ur,
	cv::Point& o_ll,
	cv::Point& o_lr,
	const std::vector<cv::Point>& corners,
	unsigned int Xoffset,
	unsigned int Yoffset)
{
	if (corners.size() != 4)
	{
		return;
	}

	// corners is arranged by: ul, ur, ll, lr

	o_ul = cv::Point(corners[0].x - Xoffset, corners[0].y - Yoffset);
	o_ur = cv::Point(corners[1].x + Xoffset, corners[1].y - Yoffset);
	o_ll = cv::Point(corners[2].x - Xoffset, corners[2].y + Yoffset);
	o_lr = cv::Point(corners[3].x + Xoffset, corners[3].y + Yoffset);

	// outer left edge; origin at lower left, Y-axis going upward
	o_l = GetInvSlope(o_ul, o_ll);

	// outer right edge; origin at lower right, Y-axis going upward
	o_r = GetInvSlope(o_ur, o_lr);

	// outer upper edge; origin at upper right, Y-axis going upwar
	o_t = GetSlope(o_ul, o_ur);

	// outer lower edge; origin at lower right, Y-axis going upward
	o_b = GetSlope(o_ll, o_lr);

} // GenerateOuterBand

//=======================================================================
void Utility::GenerateInnerBand(
	float& i_l, // slope
	float& i_r,
	float& i_t,
	float& i_b,
	cv::Point& i_ul,
	cv::Point& i_ur,
	cv::Point& i_ll,
	cv::Point& i_lr,
	const std::vector<cv::Point>& corners,
	unsigned int Xoffset,
	unsigned int Yoffset)
{
	if (corners.size() != 4)
	{
		return;
	}

	// corners is arranged by: ul, ur, ll, lr

	i_ul = cv::Point(corners[0].x + Xoffset, corners[0].y + Yoffset);
	i_ur = cv::Point(corners[1].x - Xoffset, corners[1].y + Yoffset);
	i_ll = cv::Point(corners[2].x + Xoffset, corners[2].y - Yoffset);
	i_lr = cv::Point(corners[3].x - Xoffset, corners[3].y - Yoffset);

	// inner left edge; origin at lower left, Y-axis going upward
	i_l = GetInvSlope(i_ul, i_ll);

	// inner right edge; origin at lower right, Y-axis going upward
	i_r = GetInvSlope(i_ur, i_lr);

	// inner upper edge; origin at upper right, Y-axis going upward
	i_t = GetSlope(i_ul, i_ur);

	// inner lower edge; origin at lower right, Y-axis going upwar
	i_b = GetSlope(i_ll, i_lr);
} // GenerateInnerBand

//=======================================================================
void Utility::GenerateBand(
    float& o_l, // slope
    float& o_r,
    float& o_t,
    float& o_b,
    float& i_l,
    float& i_r,
    float& i_t,
    float& i_b,
    cv::Point& o_ul,
    cv::Point& o_ur,
    cv::Point& o_ll,
    cv::Point& o_lr,
    cv::Point& i_ul,
    cv::Point& i_ur,
    cv::Point& i_ll,
    cv::Point& i_lr,
    const std::vector<cv::Point>& corners,
    unsigned int Xoffset,
	unsigned int Yoffset)
{
	GenerateOuterBand(
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

	GenerateInnerBand(
		i_l, // slope
		i_r,
		i_t,
		i_b,
		i_ul,
		i_ur,
		i_ll,
		i_lr,
		corners,
		Xoffset,
		Yoffset);

} // GenerateBand

//=======================================================================
bool Utility::IsOutsideOuter(
    unsigned int x,
    unsigned int y,
    float o_l,
    float o_r,
    float o_t,
    float o_b,
    const cv::Point& o_ur,
    const cv::Point& o_ll,
    const cv::Point& o_lr )
{
    int X, Y;
    //-------------------------------------------------
    // Outer left
    //-------------------------------------------------
    // check outer left edge;origin at lower left
    X = o_ll.x - x;
    Y = o_ll.y - y;
    const bool isLeftToOuterLeftEdge = X > Y * o_l;

    if( isLeftToOuterLeftEdge )
    {
        return true;
    }

    //-------------------------------------------------
    // Outer right
    //-------------------------------------------------
    // check outer right edge;origin at lower right
    X = o_lr.x - x;
    Y = o_lr.y - y;

    const bool isRightToOuterRightEdge = X < Y * o_r;

    if( isRightToOuterRightEdge )
    {
        return true;
    }

    //-------------------------------------------------
    // Outer top
    //------------------------------------------------
    // check outer top edge;origin at upper right
    X = x - o_ur.x;
    Y = o_ur.y - y;
    const bool isHigherThanOuterTopEdge = Y > X * o_t;

    if( isHigherThanOuterTopEdge )
    {
        return true;
    }

    //-------------------------------------------------
    // Outer bottom
    //-------------------------------------------------

    // check outer top edge;origin at lower right
    X = x - o_lr.x;
    Y = o_lr.y - y;
    const bool isLowerThanOuterBottomEdge = Y < X * o_b;

    if( isLowerThanOuterBottomEdge )
    {
        return true;
    }

    return false;
}// IsOutsideOuter

 //=======================================================================
bool Utility::IsInsideInner(
    unsigned int x,
    unsigned int y,
    float i_l,
    float i_r,
    float i_t,
    float i_b,
    const cv::Point& i_ur,
    const cv::Point& i_ll,
    const cv::Point& i_lr )
{
    int X, Y;
    //-------------------------------------------------
    // Inner left
    //-------------------------------------------------
    // check inner left edge;origin at lower left
    X = i_ll.x - x;
    Y = i_ll.y - y;
    const bool isRightToInnerLeftEdge = X < Y * i_l;

    if( !isRightToInnerLeftEdge )
    {
        return false;
    }

    //-------------------------------------------------
    // Inner right
    //-------------------------------------------------
    // check inner right edge;origin at lower right
    X = i_lr.x - x;
    Y = i_lr.y - y;
    const bool isLeftToInnerRightEdge = X > Y * i_r;

    if( !isLeftToInnerRightEdge )
    {
        return false;
    }

    //-------------------------------------------------
    // Inner top
    //-------------------------------------------------
    // check inner top edge;origin at upper right
    X = x - i_ur.x;
    Y = i_ur.y - y;
    const bool isHigherThanInnerTopEdge = Y > X * i_t;

    if( isHigherThanInnerTopEdge )
    {
        return false;
    }

    //-------------------------------------------------
    // Inner bottom
    //-------------------------------------------------
    // check inner bottom edge;origin at lower right
    X = x - i_lr.x;
    Y = i_lr.y - y;
    const bool isHigherThanInnerBottomEdge = Y > X * i_b;

    if( !isHigherThanInnerBottomEdge )
    {
        return false;
    }

    return true;

} // IsInsideInner