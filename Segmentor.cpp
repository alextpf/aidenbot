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
void Segmentor::Process(cv::Mat & input, cv::Mat & output)
{
	int kernelSize = 3;
	double std = 2.0;

	// canny low & heigh threshold
	int low = 80;
	int high = 150;

	cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), std, std);

	//std::string typeName = type2str(output.type());
	//printf("%s\n", typeName.c_str());

	cv::Mat tmp;
	output.convertTo(tmp, CV_8UC1);

#ifdef DEBUG
	cv::imshow("gauss:", output);
#endif // DEBUG

	//typeName = type2str(tmp.type());
	//printf("%s\n", typeName.c_str());

	cv::Canny(tmp, output, low, high);

#ifdef DEBUG
	cv::imshow("canny:", output);
#endif // DEBUG

	// do noise reduction
	/*dilate(output, output, cv::Mat(), cv::Point(-1, -1), NUM_ITER);
	erode(output, output, cv::Mat(), cv::Point(-1, -1), NUM_ITER );*/

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	// Find contours
	findContours(output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	// Draw contours
	cv::Mat drawing = cv::Mat::zeros(output.size(), CV_8UC3);
	cv::RNG rng(12345);

	for (int i = 0; i< contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
	}

	output = drawing;

}//Process
