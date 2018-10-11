#include "PuckFinder.h"

#define DEBUG

//===================================================================================
bool PuckFinder::FindPuck(
	Contours& contours,
	cv::Point& pt,
	const cv::Mat & input,
	const cv::Mat& mask	)
{
	// use color threshold

	// convert RGB to HSV
	cv::Mat hsvImg;
	cv::cvtColor( input, hsvImg, CV_BGR2HSV );

	const int RedLowerH = 160;
	const int RedUpperH = 179;

	const int OrangeLowerH = 0;
	const int OrangeUpperH = 20;

	const int lowerS = 160;
	const int upperS = 255;

	const int lowerV = 110;
	const int upperV = 220;

	cv::Mat maskRed;
	cv::inRange( hsvImg, cv::Scalar( RedLowerH, lowerS, lowerV ), cv::Scalar( RedUpperH, upperS, upperV ), maskRed );

	cv::Mat maskOrange;
	cv::inRange( hsvImg, cv::Scalar( OrangeLowerH, lowerS, lowerV ), cv::Scalar( OrangeUpperH, upperS, upperV ), maskOrange );

	cv::Mat res;
	cv::bitwise_or( maskRed, maskOrange, res );

#ifdef DEBUG
	cv::imshow( "res mask", res );
#endif // DEBUG

	cv::bitwise_and( res, mask, res );

#ifdef DEBUG
	cv::imshow( "res + mask", res );
#endif // DEBUG

	cv::Mat ellipse = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 5, 5 ) );

	// remove noise in background
	cv::morphologyEx( res, res, cv::MORPH_OPEN, ellipse, cv::Point( -1, -1 ), 1/*num iteration*/ );

	// remove noise in foreground
	cv::morphologyEx( res, res, cv::MORPH_CLOSE, ellipse, cv::Point( -1, -1 ), 1/*num iteration*/ );

#ifdef DEBUG
	cv::imshow( "res + mask + noise removal", res );
#endif // DEBUG

	std::vector<std::vector<cv::Point> > tmpContours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours( res, tmpContours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

	// locate puck by 1. area, 2. roundness, and 3. color(has already been used at the begining)
	for ( int i = 0; i< tmpContours.size(); i++ )
	{
		// 1. test area
		double area = cv::contourArea( tmpContours[i] );
		double areaLow = 400.0;
		double areaHigh = 520.0;
		if ( area > areaLow && area < areaHigh )
		{
			// 2. test roundness
			double perimeter = cv::arcLength( tmpContours[i], true /*is closed*/ );
			double roundness = perimeter * perimeter  *  0.78539815 / area; // if it's a circle, = 1, because perimeter = 2 * PI * r, area = PI * r^2

			if ( roundness < 20.0 && roundness > 0.05 )
			{
				// survived

#ifdef DEBUG
                cv::Mat img = cv::Mat::zeros( input.size(), CV_8U );
                cv::drawContours( img, tmpContours, i, cv::Scalar( 255 ), 2, 8, hierarchy, 0, cv::Point() );
                cv::imshow( "puck", img );
#endif // DEBUG
				contours.push_back( tmpContours[i] );
			}
		}
	} // for ( int i = 0; i< contours.size(); i++ )

	if ( contours.size() == 1 )
	{
		cv::Moments m = cv::moments( contours[0] );
		pt.x = static_cast<int>( m.m10 / m.m00 );
		pt.y = static_cast<int>( m.m01 / m.m00 );
	}

	return contours.size() == 1;
}// FindPuck