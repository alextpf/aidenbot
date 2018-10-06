#pragma once
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

#include "videoprocessor.h"
//Hue:
//	   Yellow 21 - 30
//	   Blue 75 - 130
//	   Violet 130 - 160
//	   Orange 0 - 21
//	   Green 38 - 75
//	   Red 160 - 179

//h: 0 - 1 or 178 - 179
//s: 177 - 210
//v: 167 - 190
class CheckHSV : public FrameProcessor
{
public:

    void Process( cv::Mat & input, cv::Mat & output ) override
    {
		output = input.clone();
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