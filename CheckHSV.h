#pragma once
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

#include "videoprocessor.h"

class CheckHSV : public FrameProcessor
{
public:

    void Process( cv::Mat & input, cv::Mat & output ) override
    {
        cv::cvtColor(input, input, CV_BGR2HSV );
        cv::setMouseCallback( "Input", OnMouse, &input );
        while( cv::waitKey(100) != 27 /*esc*/ )
        {
            // busy wait
        }
    }

    static void OnMouse( int event, int x, int y, int f, void* data )
    {
        cv::Mat *img = reinterpret_cast<cv::Mat*>( data );

        if( event == cv::EVENT_LBUTTONDOWN )
        {
            std::cout << "H = " << (int)img->at<cv::Vec3b>( y, x )[0];
            std::cout << "S = " << (int)img->at<cv::Vec3b>( y, x )[1];
            std::cout << "V = " << (int)img->at<cv::Vec3b>( y, x )[2] << std::endl;
        }
    }

}; // CheckHSV