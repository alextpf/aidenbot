#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// Hough transform line finder
class LineFinder
{
public:
	enum METHOD {TRAD, PROB}; // whether use traditional or probalistic Hough transform
	LineFinder();

	LineFinder(
		METHOD m,
		double dRho,
		double dTheta,
		unsigned int minVote,
		float minLength,
		float maxGap);

	~LineFinder();

	// set d_rho
	void SetDeltaRho(double r)
	{
		m_DeltaRho = r;
	}

	// set d_theta
	void SetDeltaTheta(double d)
	{
		m_DeltaTheta = d;
	}

	void SetMinVote(unsigned int v)
	{
		m_MinVote = v;
	}

	void SetMinLength(float l)
	{
		m_MinLength = l;
	}

	void SetMaxGap(float g)
	{
		m_MaxGap = g;
	}

	// main function. Hough transform to find lines
	// probalistic method
	const std::vector<cv::Vec4i>& FindLinesP(cv::Mat& binary);

	// traditional method
	const std::vector<cv::Vec2f>& FindLines(cv::Mat& binary);

	// Draw the detected lines on an image
	virtual void DrawDetectedLines(cv::Mat &image, cv::Scalar color = cv::Scalar(255, 255, 255));

	// whether to use traditional or probalistic Hough transform
	void SetMethod(METHOD m)
	{
		m_Method = m;
	}

    // remove unwanted lines by the mask
    void FilterDetectedLines(
        const std::vector<cv::Point> & corners,
        unsigned int bandWidth,
        cv::Size s );

protected:
	// whether to use traditional or probalistic Hough transform
	METHOD m_Method;

	// vector containing the end points
	// of the detected lines
	// probalistic method
	std::vector<cv::Vec4i> m_LinesP;

	// traditional method
	std::vector<cv::Vec2f> m_Lines;

	// accumulator resolution parameters
	double m_DeltaRho;
	double m_DeltaTheta;

	// minimum number of votes that a line
	// must receive before being considered
	unsigned int m_MinVote;

	// min length for a line
	float m_MinLength;

	// max allowed gap along the line
	float m_MaxGap;

}; // class LineFinder
