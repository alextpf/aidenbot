#include "Utility.h"

//=======================================================================
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
    cv::Size s,
    unsigned int offset )
{
    if( corners.size() != 4 )
    {
        return;
    }

    // corners is arranged by: ul, ur, ll, lr

    int x, y;

    x = corners[0].x - offset;
    if( x < 0 )
    {
        x = 0;
    }

    y = corners[0].y - offset;
    if( y < 0 )
    {
        y = 0;
    }

    o_ul = cv::Point( x, y );

    x = corners[1].x + offset;
    if( x >= s.width )
    {
        x = s.width - 1;
    }

    y = corners[1].y - offset;
    if( y < 0 )
    {
        y = 0;
    }

    o_ur = cv::Point( x, y );

    x = corners[2].x - offset;
    if( x < 0 )
    {
        x = 0;
    }
    y = corners[2].y + offset;
    if( y >= s.height )
    {
        y = s.height - 1;
    }

    o_ll = cv::Point( x, y );

    x = corners[3].x + offset;
    if( x >= s.width )
    {
        x = s.width - 1;
    }

    y = corners[3].y + offset;
    if( y >= s.height )
    {
        y = s.height - 1;
    }

    o_lr = cv::Point( x, y );

    // inner bound shouldn't have the out-of-image-bound issue

    i_ul = cv::Point( corners[0].x + offset, corners[0].y + offset );
    i_ur = cv::Point( corners[1].x - offset, corners[1].y + offset );
    i_ll = cv::Point( corners[2].x + offset, corners[2].y - offset );
    i_lr = cv::Point( corners[3].x - offset, corners[3].y - offset );

    //-------------------------------------------
    // Outer
    //-------------------------------------------
    // outer left edge; origin at lower left, Y-axis going upward
    o_l = GetInvSlope( o_ul, o_ll );

    // outer right edge; origin at lower right, Y-axis going upward
    o_r = GetInvSlope( o_ur, o_lr );

    // outer upper edge; origin at upper right, Y-axis going upwar
    o_t = GetSlope( o_ul, o_ur );

    // outer lower edge; origin at lower right, Y-axis going upward
    o_b = GetSlope( o_ll, o_lr );

    //-------------------------------------------
    // Inner
    //-------------------------------------------
    // inner left edge; origin at lower left, Y-axis going upward
    i_l = GetInvSlope( i_ul, i_ll );

    // inner right edge; origin at lower right, Y-axis going upward
    i_r = GetInvSlope( i_ur, i_lr );

    // inner upper edge; origin at upper right, Y-axis going upward
    i_t = GetSlope( i_ul, i_ur );

    // inner lower edge; origin at lower right, Y-axis going upwar
    i_b = GetSlope( i_ll, i_lr );
    //-------------------------------------------
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