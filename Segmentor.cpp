#include "Segmentor.h"
#include "TableFinder.h"
#include "Utility.h"

#define PI							3.1415926
#define DEG_TO_RAD					PI / 180.0f

// color definition
#define BLUE   cv::Scalar(255, 0, 0) //BGR
#define GREEN  cv::Scalar(0, 255, 0)
#define RED    cv::Scalar(0, 0, 255)
#define YELLOW cv::Scalar(0, 255, 255)
#define WHITE  cv::Scalar(255, 255, 255)
#define BLACK  cv::Scalar(0,0,0)

#define DEBUG
//#define DEBUG_CORNER

//=======================================================================
// helper function to show type
std::string type2str(int type)
{
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}//std::string type2str(int type)

//=======================================================================
Segmentor::Segmentor()
: m_TableFound( false )
, m_BandWidth( 0 )
{}

//=======================================================================
void Segmentor::OnMouse(int event, int x, int y, int f, void* data)
{
	std::vector<cv::Point> *curobj = reinterpret_cast<std::vector<cv::Point>*>(data);

	if (event == cv::EVENT_LBUTTONDOWN)
	{
		curobj->push_back( cv::Point( x, y ) );
	}
}

//=======================================================================
void Segmentor::Process(cv::Mat & input, cv::Mat & output)
{
	if (!m_TableFound)
	{
		// blur image first by Gaussian
		int kernelSize = 3;
		double std = 2.0;

		cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), std, std);

#ifdef DEBUG
		cv::imshow("gauss:", output);
#endif // DEBUG
		bool doThreshold = false; // later I figured it's not of great use in our case
		if (doThreshold)
		{
			//std::string typeName = type2str(output.type());
			//printf("%s\n", typeName.c_str());

			// convert to gray scale
			cv::Mat tmp;
			cv::cvtColor(output, tmp, cv::COLOR_RGB2GRAY);

			/*std::string typeName = type2str(tmp.type());
			printf("%s\n", typeName.c_str());*/

			// adaptive threshold
			int adaptiveMethod = cv::ADAPTIVE_THRESH_MEAN_C;
			int thresholdType = cv::THRESH_BINARY;
			int blockSiz = 55;

			cv::Mat tmp1;

			cv::adaptiveThreshold(tmp, tmp1, 255, adaptiveMethod, thresholdType, blockSiz, 5);
			//cv::threshold(tmp, tmp1, 128, 255, cv::THRESH_BINARY_INV);

			//typeName = type2str(tmp1.type());
			//printf("%s\n", typeName.c_str());
#ifdef DEBUG
			cv::imshow("adaptiveThreshold:", tmp1);
#endif // DEBUG
		}//if (doThreshold)

#ifndef DEBUG_CORNER
		cv::imshow("Input", input);
		cv::setMouseCallback("Input", OnMouse, &m_Corners);
		while (m_Corners.size() < 4)
		{
			unsigned m = m_Corners.size();
			if ( m > 0)
			{
				cv::circle(input, m_Corners[m - 1], 3, GREEN, 2);
			}

			cv::imshow("Input", input);
			cv::waitKey(10);
		}

		// last point
		cv::circle(input, m_Corners[3], 3, GREEN, 2);

		cv::imshow("Input", input);
		cv::waitKey(10);
#else
		cv::Point pt1(69, 26);
		cv::Point pt2(21, 324);
		cv::Point pt3(632, 312);
		cv::Point pt4(572, 18);

		m_Corners.push_back(pt1);
		m_Corners.push_back(pt2);
		m_Corners.push_back(pt3);
		m_Corners.push_back(pt4);
#endif
		// order the 4 corners
        OrderCorners();

#ifdef DEBUG
		// draw the bands around 4 picked corners
		// m_Corners is arranged by: ul, ur, ll, lr
		cv::circle(input, m_Corners[0], 3, GREEN, 2);
		cv::circle(input, m_Corners[1], 3, RED, 2);
		cv::circle(input, m_Corners[2], 3, BLUE, 2);
		cv::circle(input, m_Corners[3], 3, WHITE, 2);

		cv::imshow("ORder:", input);
#endif // DEBUG

		// canny low & heigh threshold
		int low = 50;
		int high = 100;

		// convert to gray scale
		cv::Mat tmp;
		cv::cvtColor(output, tmp, cv::COLOR_RGB2GRAY);

		cv::Canny(tmp, output, low, high);

        // dilate Canny results
        cv::dilate(output, output, cv::Mat(), cv::Point( -1, -1 ), 1 /*num iteration*/ );

#ifdef DEBUG
		cv::imshow("canny:", output);
#endif // DEBUG

		// mask out anything that's outside of the user-picked band
		MaskCanny(output);

#ifdef DEBUG
        cv::Mat copy = input.clone();

        // draw bounds
        cv::Scalar color = GREEN; // green

        cv::line( copy, m_o_ul, m_o_ur, color );
        cv::line( copy, m_o_ul, m_o_ll, color );
        cv::line( copy, m_o_ur, m_o_lr, color );
        cv::line( copy, m_o_ll, m_o_lr, color );

        cv::line( copy, m_i_ul, m_i_ur, color );
        cv::line( copy, m_i_ul, m_i_ll, color );
        cv::line( copy, m_i_ur, m_i_lr, color );
        cv::line( copy, m_i_ll, m_i_lr, color );

        cv::imshow( "bound:", copy );
#endif // DEBUG

		// Hough line transform
		double dRho = 1.0f;
		double dTheta = CV_PI / 180.0f;
		unsigned int minVote = 60;//360 / 2;
		float minLength = 50.0f;// 360 / 2;
		float maxGap = 60.0f;
		//LineFinder::METHOD m = LineFinder::METHOD::TRAD;
        LineFinder::METHOD m = LineFinder::METHOD::PROB;

		TableFinder tableFinder(m, dRho, dTheta, minVote, minLength, maxGap);

        if( m == LineFinder::METHOD::PROB )
        {
            const std::vector<cv::Vec4i>& lines = tableFinder.FindLinesP( output );

#ifdef DEBUG
            //cv::Mat copy = input.clone();
			tableFinder.DrawDetectedLines( input, BLUE);
			cv::imshow("HoughLine:", input );
#endif // DEBUG

            // filter out the lines that's out of bound
			tableFinder.Refine4Edges( m_Corners, m_BandWidth, input );
        }
        else
        {
            const std::vector<cv::Vec2f>& lines = tableFinder.FindLines( output );
        }

#ifdef DEBUG
		tableFinder.DrawTableLines( input, RED );
		cv::imshow("Filtered HoughLine:", input );
#endif // DEBUG

		m_TableFound = true;
	}

}//Process

//=======================================================================
void Segmentor::OrderCorners()
{
	if (m_Corners.size() != 4)
	{
		return;
	}

	cv::Point left1(100000, 0); // left most
	cv::Point left2(100000, 0); // 2nd to left most
	cv::Point right1(-1, 0); // right most
	cv::Point right2(-1, 0); // 2nd to right most

	for (int i = 0; i < 4; i++)
	{
		if (m_Corners[i].x < left1.x)
		{
			left2 = left1;
			left1 = m_Corners[i];
		}
		else if (m_Corners[i].x < left2.x)
		{
			left2 = m_Corners[i];
		}

		if (m_Corners[i].x > right1.x)
		{
			right2 = right1;
			right1 = m_Corners[i];
		}
		else if (m_Corners[i].x > right2.x)
		{
			right2 = m_Corners[i];
		}
	} // for i = 1 - 4

	// determine upper and lower
	// m_Corners is arranged by: ul, ur, ll, lr

	if (left1.y > left2.y)
	{
		// left1 is lower left, left2 is upper left
		m_Corners[0] = left2;
		m_Corners[2] = left1;
	}
	else
	{
		// left2 is lower left, left1 is upper left
		m_Corners[0] = left1;
		m_Corners[2] = left2;
	}

	if (right1.y > right2.y)
	{
		// right1 is lower right, right2 is upper right
		m_Corners[1] = right2;
		m_Corners[3] = right1;
	}
	else
	{
		// right2 is lower right, right1 is upper right
		m_Corners[1] = right1;
		m_Corners[3] = right2;
	}

}// OrderCorners

//=======================================================================
void Segmentor::MaskCanny(cv::Mat & img)
{
    // Generate a band around user-picked 4 corners.
    // This band is wider than the m_BandWidth because
    // such that later in Hough transform, it doesn't
    // detect the artificial line generated by the masking.
    // After Hough transform line detection, we then
    // mask again using the true m_BandWidth
    cv::Size s = img.size();

    float o_l, o_r, o_t, o_b;
    float i_l, i_r, i_t, i_b;

    const unsigned int offset = m_BandWidth + 10;

    Utility::GenerateBand(
        o_l, o_r, o_t, o_b,
        i_l, i_r, i_t, i_b,
        m_o_ul, m_o_ur, m_o_ll, m_o_lr,
        m_i_ul, m_i_ur, m_i_ll, m_i_lr,
        m_Corners, offset, offset );

	for (unsigned int i = 0; i < static_cast<unsigned int>(s.width); i++)
	{
		for (unsigned int j = 0; j < static_cast<unsigned int>(s.height); j++)
		{
			const bool isOutsideOuter = Utility::IsOutsideOuter( i, j, o_l, o_r, o_t, o_b, m_o_ur, m_o_ll, m_o_lr );
			if ( isOutsideOuter )
			{
				img.at<uchar>(j, i) = 0;
			}
			else
			{
				const bool isInsideInner = Utility::IsInsideInner(i, j, i_l, i_r, i_t, i_b, m_i_ur, m_i_ll, m_i_lr );
				if ( isInsideInner )
				{
					img.at<uchar>(j, i) = 0;
				}
			}
		}
	}

#ifdef DEBUG
	cv::imshow("masked canny:", img );
#endif // DEBUG

}//void MaskCanny(cv::Mat & img);