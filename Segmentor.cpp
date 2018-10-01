#include "Segmentor.h"

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
		int kernelSize = 5;
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


#ifdef DEBUG
		// draw bounds
		cv::Scalar color = GREEN; // green

		cv::line(input, m_o_ul, m_o_ur, color);
		cv::line(input, m_o_ul, m_o_ll, color);
		cv::line(input, m_o_ur, m_o_lr, color);
		cv::line(input, m_o_ll, m_o_lr, color);

		cv::line(input, m_i_ul, m_i_ur, color);
		cv::line(input, m_i_ul, m_i_ll, color);
		cv::line(input, m_i_ur, m_i_lr, color);
		cv::line(input, m_i_ll, m_i_lr, color);

		cv::imshow("bound:", input);
#endif // DEBUG

		// canny low & heigh threshold
		int low = 50;
		int high = 100;


		// convert to gray scale
		cv::Mat tmp;
		cv::cvtColor(output, tmp, cv::COLOR_RGB2GRAY);

		cv::Canny(tmp, output, low, high);

#ifdef DEBUG
		cv::imshow("canny:", output);
#endif // DEBUG

		MaskCanny(output);

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

	// m_Corners is arranged by: ul, ur, ll, lr
	
	m_o_ul = cv::Point(m_Corners[0].x - m_BandWidth, m_Corners[0].y - m_BandWidth);
	m_o_ur = cv::Point(m_Corners[1].x + m_BandWidth, m_Corners[1].y - m_BandWidth);
	m_o_ll = cv::Point(m_Corners[2].x - m_BandWidth, m_Corners[2].y + m_BandWidth);
	m_o_lr = cv::Point(m_Corners[3].x + m_BandWidth, m_Corners[3].y + m_BandWidth);

	m_i_ul = cv::Point(m_Corners[0].x + m_BandWidth, m_Corners[0].y + m_BandWidth);
	m_i_ur = cv::Point(m_Corners[1].x - m_BandWidth, m_Corners[1].y + m_BandWidth);
	m_i_ll = cv::Point(m_Corners[2].x + m_BandWidth, m_Corners[2].y - m_BandWidth);
	m_i_lr = cv::Point(m_Corners[3].x - m_BandWidth, m_Corners[3].y - m_BandWidth);

}// OrderCorners

//=======================================================================
float GetSlope(const cv::Point& p1, const cv::Point& p2)
{
	int dx = p2.x - p1.x;
	int dy = p1.y - p2.y;

	if (dy == 0)
	{
		return 100000.0f;
	}
	else
	{
		return (float)dy / (float)dx; // line equation: y = x * slope
	}
} // GetSlope

//=======================================================================
bool Segmentor::IsOutsideOuter(
	unsigned int x, 
	unsigned int y, 
	float o_l, 
	float o_r, 
	float o_u, 
	float o_d)
{
	//-------------------------------------------------
	// Outer left
	//-------------------------------------------------
	bool isLeftToOuterLeftEdge = false;

	// check outer left edge;origin at lower left
	if (o_l > 10000)
	{
		//vertical edge
		isLeftToOuterLeftEdge = x < m_o_ll.x;
	}
	else
	{
		x = x - m_o_ll.x;
		y = m_o_ll.y - y;

		if (o_l > 0)
		{
			// upper_left is to the right of lower_left
			isLeftToOuterLeftEdge = y > x * o_l;
		}
		else
		{
			// upper_left is to the left of lower_left
			isLeftToOuterLeftEdge = y < x * o_l;
		}
	}

	if (isLeftToOuterLeftEdge)
	{
		return true;
	}

	//-------------------------------------------------
	// Outer right
	//-------------------------------------------------
	bool isRightToOuterRightEdge = false;

	// check outer right edge;origin at lower right
	if (o_r > 10000)
	{
		isRightToOuterRightEdge = x > m_o_lr.x;
	}
	else
	{
		x = x - m_o_lr.x;
		y = m_o_lr.y - y;

		if (o_r > 0)
		{
			// upper_right is to the right of lower_right
			isRightToOuterRightEdge = y < x * o_r;
		}
		else
		{
			// upper_right is to the left of lower_right
			isRightToOuterRightEdge = y > x * o_r;
		}
	}

	if (isRightToOuterRightEdge)
	{
		return true;
	}

	//-------------------------------------------------
	// Outer top
	//-------------------------------------------------
	
	// check outer top edge;origin at lower right
	x = x - m_o_ur.x;
	y = m_o_ur.y - y;
	bool isHigherThanOuterTopEdge = y > x * o_u;

	if (isHigherThanOuterTopEdge)
	{
		return true;
	}

	//-------------------------------------------------
	// Outer bottom
	//-------------------------------------------------

	// check outer top edge;origin at lower right
	x = x - m_o_lr.x;
	y = m_o_lr.y - y;
	bool isLowerThanOuterBottomEdge = y < x * o_d;

	if (isLowerThanOuterBottomEdge)
	{
		return true;
	}

}// IsOutsideOuter

//=======================================================================
void Segmentor::MaskCanny(cv::Mat & img)
{	
	//-------------------------------------------
	// Outer
	//-------------------------------------------
	// outer left edge; origin at lower left, Y-axis going upward
	float o_l = GetSlope(m_o_ul, m_o_ul);
	
	// outer right edge; origin at lower right, Y-axis going upward
	float o_r = GetSlope(m_o_ur, m_o_lr);

	// outer upper edge; origin at upper right, Y-axis going upwar	
	float o_u = GetSlope(m_o_ul, m_o_ur);

	// outer lower edge; origin at lower right, Y-axis going upward	
	float o_d = GetSlope(m_o_ll, m_o_lr);

	//-------------------------------------------
	// Inner
	//-------------------------------------------
	// inner left edge; origin at lower left, Y-axis going upward	
	float i_l = GetSlope(m_i_ul, m_i_ll);

	// inner right edge; origin at lower right, Y-axis going upward	
	float i_r = GetSlope(m_i_ur, m_i_lr);

	// inner upper edge; origin at upper right, Y-axis going upward	
	float i_u = GetSlope(m_i_ul, m_i_ur);

	// inner lower edge; origin at lower right, Y-axis going upwar	
	float i_d = GetSlope(m_i_ll, m_i_lr);
	//-------------------------------------------
	cv::Size s = img.size();

	int x, y;// converted coordinate
	
	for (unsigned int i = 0; i < static_cast<unsigned int>(s.width); i++)
	{
		for (unsigned int j = 0; j < static_cast<unsigned int>(s.height); j++)
		{
			bool isOutSideOuter = IsOutsideOuter(i, j, o_l, o_r, o_u, o_d);
			
			if (isOutOfBound)
			{
				output.at<uchar>(j, i) = 0;
			}
		}
	}
#ifdef DEBUG
	cv::imshow("masked canny:", output);
#endif // DEBUG
	
}//void MaskCanny(cv::Mat & img);