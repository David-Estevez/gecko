#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../libraries/HandDetector.h"
#include "../libraries/HandDescription.h"
#include "../libraries/handUtils.h"
#include "../libraries/FingerDetector.h"
#include "../libraries/mouse.h"

int main( int argc, char * argv[] )
{
    //-- Setup video
    //--------------------------------------------------------------
    cv::Mat subs;
    cv::VideoCapture cap;

    //-- Open video source
    if ( argc > 1)
    {
		cap.open( argv[1] );
    }
    else
    {
		cap.open(-1);
		cap.set(CV_CAP_PROP_BRIGHTNESS, 0.5);
    }

    //-- Check if open
    if ( !cap.isOpened() )
    {
		std::cerr << "Device could not be opened." << std::endl;
		return(1);
    }

    //-- Get frame rate
//    double rate = cap.get( CV_CAP_PROP_FPS);
//    int delay = 1000/rate;
    int delay = 24; //-- Force 24 s delay


    //-- Declare variables
    //--------------------------------------------------------------------
    //-- Program control
    bool stop = false;
    int debugValue = 0;

    //-- To find the hand
    HandDetector handDetector;



	//-- Object that will store the parameters of the hand
	HandDescription hand_descriptor;
	
	
	//-- Mouse object to control the mouse 
	Mouse mouse; 

	//-- Calibration loop
	
	handDetector.calibrationLoop(cap);

    //-- Main loop
    //--------------------------------------------------------------------
    stop = false;
    while(!stop)
    {
	//------------------------------------------------------------------------------------------------------
	//-- Get current frame
	//-------------------------------------------------------------------------------------------------------
	cv::Mat frame, display;
	if ( ! cap.read( frame ) )
	    break;
	    
	cv::flip(frame,frame,1);

	//-- Remove the background
	frame = sBackgroundSubs(frame);
	
	
	//------------------------------------------------------------------------------------------------------
	//-- Process it
	//------------------------------------------------------------------------------------------------------
	cv::Mat processed;
	switch( debugValue )
	{
	    case 0: case 2:
			handDetector.calibrate();
			handDetector( frame, processed);
		break;

	    case 1:
			handDetector.calibrate( handDetector.getLower(), handDetector.getUpper());
			handDetector( frame, processed);
		break;

	    default:
			processed = frame;
		break;
	   }


//-- Contour extraction
	hand_descriptor.contourExtraction(frame, processed); 


//-- Hand's angle
	std::cout << "[" << hand_descriptor.getHandAngle() << "]" << std::endl;



	//--------------------------------------------------------------------------------------------------
	//-- Adding text:
	//--------------------------------------------------------------------------------------------------
	std::stringstream ss;
	ss << "Mode: ";

	switch( debugValue)
	{
	    case 0:
	    ss << "Default values";
	    break;

	    case 1:
	    ss << "Custom values->" << handDetector.getLower() << " " << handDetector.getUpper();
	    break;

	    case 2:
	    ss << "Tracking hand";
	    break;

	}
	ss << " Angle: " <<hand_descriptor.getHandAngle()<< "";
	std::string text = ss.str();
	cv::putText( display, text.c_str(), cv::Point(0, 18),
		     cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(0, 0, 255));
 

//--	Move Cursor 

	if ( debugValue == 2)
		mouse.moveCursor(frame, hand_descriptor.getContours()); 
		

	hand_descriptor.angleControl();

	//-----------------------------------------------------------------------------------------------------
	//-- Decide what to do next depending on key pressed
	//-----------------------------------------------------------------------------------------------------
	char key = (char) cv::waitKey( delay);
	switch( key)
	{
	    case 'f': //-- Filtered frame
		debugValue = 0;
		break;
	    case 'd': //-- hardcoded filter
		debugValue = 1;
		break;
	    case 'k': //-- Move mouse
		debugValue = 2;
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
