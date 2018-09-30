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
, m_OuterXOffset( 0 )
, m_OuterYOffset( 0 )
, m_InnerXOffset( 0 )
, m_InnerYOffset( 0 )
{}

void Segmentor::SetXOutterOffset(unsigned int x)
{
	m_OuterXOffset = x;
}

void Segmentor::SetYOutterOffset(unsigned int y)
{
	m_OuterYOffset = y;
}

void Segmentor::SetXInnerOffset(unsigned int x)
{
	m_InnerXOffset = x;
}

void Segmentor::SetYInnerOffset(unsigned int y)
{
	m_InnerYOffset = y;
}

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
			for (int i = 0; i < m_Corners.size(); i++)
			{
				cv::circle(input, m_Corners[i], 3, GREEN, 2);
			}
			cv::imshow("Input", input);
			cv::waitKey(10);
		}

		cv::circle(input, m_Corners[3], 3, GREEN, 2);
		
		cv::imshow("Input", input);
		cv::waitKey(10);
		
//
//		// canny low & heigh threshold
//		int low = 50;
//		int high = 100;
//
//
//		//typeName = type2str(tmp.type());
//		//printf("%s\n", typeName.c_str());
//
//		cv::Canny(tmp, output, low, high);
//
//#ifdef DEBUG
//		cv::imshow("canny:", output);
//#endif // DEBUG
//
//#ifdef DEBUG
//		// draw bounds
//		//cv::Mat clone = input.clone();
//
//		cv::Scalar color = GREEN; // green
//		cv::Size s = input.size();
//
//		// outer
//		cv::Point u_ul( m_OuterXOffset,				m_OuterYOffset );
//		cv::Point u_ur( s.width - m_OuterXOffset,	m_OuterYOffset);
//		cv::Point u_ll( m_OuterXOffset,				s.height - m_OuterYOffset);
//		cv::Point u_lr( s.width - m_OuterXOffset,	s.height - m_OuterYOffset);
//		
//		cv::line(input, u_ul, u_ur, color);
//		cv::line(input, u_ul, u_ll, color);
//		cv::line(input, u_ur, u_lr, color);
//		cv::line(input, u_ll, u_lr, color);
//
//		// inner
//		cv::Point i_ul( m_InnerXOffset,				m_InnerYOffset );
//		cv::Point i_ur( s.width - m_InnerXOffset,	m_InnerYOffset );
//		cv::Point i_ll( m_InnerXOffset,				s.height - m_InnerYOffset );
//		cv::Point i_lr( s.width - m_InnerXOffset,	s.height - m_InnerYOffset );
//
//		cv::line(input, i_ul, i_ur, color);
//		cv::line(input, i_ul, i_ll, color);
//		cv::line(input, i_ur, i_lr, color);
//		cv::line(input, i_ll, i_lr, color);
//
//		cv::imshow("bound:", input );
//#endif // DEBUG
//
//		// do masking
//		if (m_OuterXOffset != 0 || m_OuterYOffset != 0 ||
//			m_InnerXOffset != 0 || m_InnerYOffset != 0)
//		{
//			cv::Size s = input.size();
//
//			for (unsigned int i = 0; i < static_cast<unsigned int>(s.width); i++)
//			{
//				for (unsigned int j = 0; j < static_cast<unsigned int>(s.height); j++)
//				{
//					bool isOutOfBound = 
//						( i < m_OuterXOffset || i > s.width - m_OuterXOffset || // outer region
//						  j < m_OuterYOffset || j > s.height - m_OuterYOffset ) ||
//					    ( i > m_InnerXOffset && i < s.width - m_InnerXOffset && // inner region
//						  j > m_InnerYOffset && j < s.height - m_InnerYOffset );
//
//					if ( isOutOfBound )
//					{
//						output.at<uchar>(j,i) = 0;
//					}
//				}
//			}
//		}
//
//#ifdef DEBUG
//		cv::imshow("masked canny:", output);
//#endif // DEBUG
//
		m_TableFound = true;
	}

}//Process
