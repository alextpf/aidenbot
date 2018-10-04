#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class Utility
{
public:
	// find the intersection between line l1 & l2,
	// returns the intersection as well as the slope
	// of l1 & l2
	// coordinate systems is the same as image coordinate
	// i.e. X going right, Y going downward
	// if no intersection, return false
	static bool FindLineIntersection( 
		const cv::Vec4i& l1, 
		const cv::Vec4i& l2,
		cv::Point& intr,
		float& s1, // slope
		float& s2 );

    // Generate a band around the user-picked 4 corners,
    // with width offset.
    // @return the slope of the outer and inner quadrilateral
    // @note the slope for top & bottom edges is in the corrdinate that X-axis is horizontal, Y-axis vertical
    // but the slope for left & right edges is in the corrdinate that Y-axis is horizontal, X-axis vertical
    // The inner and outer quadrilateral corners are also cached in member data m_o_XX, and m_i_XX
    static void GenerateBand(
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
		unsigned int Yoffset);

	//=======================================================================
	static void Utility::GenerateOuterBand(
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
		unsigned int Yoffset);

	//=======================================================================
	static void Utility::GenerateInnerBand(
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
		unsigned int Yoffset);

    //=======================================================================
    // is the point (in image coordiante) outside outer bound
    //=======================================================================
    static bool IsOutsideOuter(
        unsigned int x,
        unsigned int y,
        float o_l,
        float o_r,
        float o_t,
        float o_b,
        const cv::Point& o_ur,
        const cv::Point& o_ll,
        const cv::Point& o_lr );

    //=======================================================================
    // is the point (in image coordiante) inside inner bound
    //=======================================================================
    static bool IsInsideInner(
        unsigned int x,
        unsigned int y,
        float i_l,
        float i_r,
        float i_t,
        float i_b,
        const cv::Point& i_ur,
        const cv::Point& i_ll,
        const cv::Point& i_lr );
}; // Utility