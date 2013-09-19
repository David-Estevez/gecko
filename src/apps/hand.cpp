#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../libraries/HandDetector.h"

void processFrame( cv::Mat& src, cv::Mat& dst, int debug = 0)
{
    if (debug == 0 )
    {
    //canny( src, dst);
    //substractor(src, dst);

    cv::cvtColor( src, dst, CV_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split( dst, channels);
    cv::Mat hue = channels[0];
    //cv::threshold( hue, hue, 30, 120, cv::THRESH_BINARY);
    //cv::inRange(hsv, Scalar(0, 58, 89), Scalar(25, 173, 229), bw);
    dst = hue;
    }
    else if (debug == 1)
    {
	//-- Do nothing (is this possible?)
	return;
    }
}


int main( int argc, char * argv[] )
{
    //-- Setup video
    //--------------------------------------------------------------

    cv::VideoCapture webcam;

    //-- Open video source
    if ( argc > 1)
    {
	webcam.open( argv[1] );
    }
    else
    {
	webcam.open(-1);
	webcam.set(CV_CAP_PROP_BRIGHTNESS, 0.5);
    }

    //-- Check if open
    if ( !webcam.isOpened() )
    {
	std::cerr << "Device could not be opened." << std::endl;
	return(1);
    }

    //-- Get frame rate
    double rate = webcam.get( CV_CAP_PROP_FPS);
    int delay = 1000/rate;
    delay = 24; //-- Force 24 FPS


    //-- Declare variables
    //--------------------------------------------------------------------
    bool stop = false;
    int debugValue = 1;

   // cv::BackgroundSubtractorMOG2 substractor;
   // substractor.bShadowDetection = false;


    //-- Main loop
    //--------------------------------------------------------------------
    while(!stop)
    {
	//-- Get current frame
	cv::Mat frame;
	if ( ! webcam.read( frame ) )
	    break;

	//-- Process it
	cv::Mat processed = frame;
	processFrame( frame, processed, debugValue);

	//-- Show processed image
	cv::imshow( "Processed Stream", processed);


	//-- Decide what to do next depending on key pressed
	char key = (char) cv::waitKey( delay);
	switch( key)
	{
	    case 'f': //-- Filtered
		debugValue = 0;
		break;
	    case 'b': //-- By-pass filters
		debugValue = 1;
		break;
	    case (char) 27:
		stop = true;
		break;
	    default:
		continue;
	}
    }

return 0;
}



