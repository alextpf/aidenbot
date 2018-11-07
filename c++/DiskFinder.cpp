#include "DiskFinder.h"

//#define DEBUG
//===================================================================================
bool DiskFinder::FindDisk1Thresh(
	Contours& contours,
	cv::Point& center,
	const cv::Mat& hsvImg,
	const cv::Vec6i& thresh,
	const cv::Mat& mask )
{
	// use color threshold

	cv::Mat res;
	cv::inRange( hsvImg, cv::Scalar( thresh[0], thresh[1], thresh[2] ), cv::Scalar( thresh[3], thresh[4], thresh[5] ), res );

#ifdef DEBUG
	cv::imshow( "res mask", res );
#endif // DEBUG

	if ( !mask.empty() )
	{
		cv::bitwise_and( res, mask, res );
	}

#ifdef DEBUG
	cv::imshow( "res + mask", res );
#endif // DEBUG

	return FindDiskInternal( contours, center, res );

}// FindDisk1Thresh

//===================================================================================
bool DiskFinder::FindDisk2Thresh(
	Contours& contours,
	cv::Point& center,
	const cv::Mat& hsvImg,
	const cv::Vec6i& thresh1,
	const cv::Vec6i& thresh2,
	const cv::Mat& mask )
{
	// use color threshold
	cv::Mat mask1;
	cv::inRange( hsvImg, cv::Scalar( thresh1[0], thresh1[1], thresh1[2] ), cv::Scalar( thresh1[3], thresh1[4], thresh1[5] ), mask1 );

	cv::Mat mask2;
	cv::inRange( hsvImg, cv::Scalar( thresh2[0], thresh2[1], thresh2[2] ), cv::Scalar( thresh2[3], thresh2[4], thresh2[5] ), mask2 );

	cv::Mat res;
	cv::bitwise_or( mask1, mask2, res );

#ifdef DEBUG
	cv::imshow( "res mask", res );
#endif // DEBUG

	if ( !mask.empty() )
	{
		cv::bitwise_and( res, mask, res );
	}

#ifdef DEBUG
	cv::imshow( "res + mask", res );
#endif // DEBUG

	return FindDiskInternal( contours, center, res );

}// FindDisk2Thresh

//===================================================================================
bool DiskFinder::FindDiskInternal(
	Contours& contours,
	cv::Point& center,
	const cv::Mat& mask )
{
	cv::Mat ellipse = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 5, 5 ) );

	// remove noise in background
	cv::morphologyEx( mask, mask, cv::MORPH_OPEN, ellipse, cv::Point( -1, -1 ), 1/*num iteration*/ );

	// remove noise in foreground
	cv::morphologyEx( mask, mask, cv::MORPH_CLOSE, ellipse, cv::Point( -1, -1 ), 1/*num iteration*/ );

#ifdef DEBUG
	cv::imshow( "res + mask + noise removal", mask );
#endif // DEBUG

	std::vector<std::vector<cv::Point> > tmpContours;
	std::vector<std::vector<cv::Point> > tmpContours2;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<double> roundness;

	cv::findContours( mask, tmpContours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

	// locate puck by 1. area, 2. roundness, and 3. color(has already been used at the begining)
	for ( int i = 0; i< tmpContours.size(); i++ )
	{
		// 1. test area
		double area = cv::contourArea( tmpContours[i] );
		if ( area > m_AreaLow && area < m_AreaHigh )
		{
			// 2. test roundness
			double perimeter = cv::arcLength( tmpContours[i], true /*is closed*/ );
			double tmpRoundness = perimeter * perimeter  *  0.78539815 / area; // if it's a circle, = 1, because perimeter = 2 * PI * r, area = PI * r^2

			if ( tmpRoundness < 20.0 && tmpRoundness > 0.05 )
			{
				// survived
				tmpContours2.push_back( tmpContours[i] );
				roundness.push_back( tmpRoundness );
			}
		}
	} // for ( int i = 0; i< puckContours.size(); i++ )

	if ( tmpContours2.size() == 1 )
	{
		contours.push_back( tmpContours2[0] );
		cv::Moments m = cv::moments( contours[0] );
		center.x = static_cast<int>( m.m10 / m.m00 );
		center.y = static_cast<int>( m.m01 / m.m00 );
	}
	else if ( tmpContours2.size() > 1 )
	{
		// choose the one that has the closest-to-1 roundness
		double minDiff = 100000.0;
		int idx = -1;
		for ( int i = 0; i < tmpContours2.size(); i++ )
		{
			double absDif = std::abs( roundness[i] - 1.0 );
			if ( absDif < minDiff )
			{
				idx = i;
				minDiff = absDif;
			}
		} // for
		contours.push_back( tmpContours2[idx] );

		cv::Moments m = cv::moments( contours[0] );
		center.x = static_cast<int>( m.m10 / m.m00 );
		center.y = static_cast<int>( m.m01 / m.m00 );
	}

	return contours.size() == 1;
} // FindDiskInternal
