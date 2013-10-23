#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../libraries/HandDetector.h"
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
    double rate = cap.get( CV_CAP_PROP_FPS);
    int delay = 1000/rate;
    delay = 24; //-- Force 24 s delay


    //-- Declare variables
    //--------------------------------------------------------------------
    //-- Program control
    bool stop = false;
    int debugValue = 0;

    //-- To find the hand
    HandDetector handDetector;
    cv::Scalar lower, upper;
    static const int halfSide = 40;

    //-- Angle of hand
    float handAngle = 0;

    //-- Calibration loop
    //--------------------------------------------------------------------
    cv::namedWindow( "Calibrating skin", cv::WINDOW_AUTOSIZE);

    while( !stop)
    {
	//-- Get current frame
	cv::Mat frame, cal_screen;
	if (! cap.read( frame ) )
	    break;
	cv::flip(frame,frame,1);

	//-- Add calibration frame
	drawCalibrationMarks(frame, cal_screen, halfSide);

	//-- Show calibration screen
	cv::imshow( "Calibrating skin", cal_screen);

	//-- Wait for user confirmation
	char key =  cv::waitKey(delay);
	if ( key == 10 || key == 13 )
	{
	    //-- Get region of interest data:
	    int image_rows = frame.rows;
	    int image_cols = frame.cols;

	    cv::Mat ROI = frame( cv::Rect( cv::Point( image_cols / 2 - halfSide,  image_rows/2 - halfSide ),
					   cv::Point( image_cols / 2 + halfSide,  image_rows/2 + halfSide)));
	    //cv::imshow( "Test", ROI);

	    handDetector.calibrate( ROI );
	    handDetector.getCalibration( lower, upper);

	    //drawHistogramHSV( ROI );

	    //-- Close window
	    cvDestroyWindow( "Calibrating skin");
	    break;
	}
    }

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
		handDetector.calibrate( lower, upper);
		handDetector( frame, processed);
		break;

	    default:
		processed = frame;
		break;
	   }

	//----------------------------------------------------------------------------------------------------
	//-- Process data and print it on screen:
	//----------------------------------------------------------------------------------------------------
	//-- Obtain contours:
	std::vector< std::vector<cv::Point> > handContour;
	filteredContour( processed, handContour);

	if ( display.total() == 0)
	    display = frame.clone();

	if ( (int) handContour.size() > 0 )
	{
	    //-- Draw contours:
	    cv::drawContours( display, handContour, 0, cv::Scalar( 0, 0, 255), 1, 8);
	    //cv::fillConvexPoly( dst, contours[largestId], cv::Scalar( 255, 255, 255));

	    //-- Draw rotated rectangle:
	    cv::RotatedRect minRect = cv::minAreaRect( handContour[0]);
	    cv::Point2f rect_points[4]; minRect.points( rect_points );
	    for( int j = 0; j < 4; j++ )
		cv::line( display, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255, 0, 0) , 1, 8 );

	    double newHandAngle = getAngle(minRect);
	    handAngle = newHandAngle < 0 ? handAngle : newHandAngle;
	    std::cout << "[" << handAngle << "]" << std::endl;


	    //-- Show hand ROI
	    cv::Rect rect  =  cv::boundingRect(handContour[0]);
	    cv::Mat ROI_hand = frame( rect).clone();
	    cv::imshow("hand", ROI_hand);
	    
	    //-- Finger detector 
	    /*
	    int fingers=0; 
	    FingerDetector (ROI_hand, fingers);
	    
	    cv::imshow("fingers", ROI_hand);*/

	}
	else
	    std::cerr << "No contours found!" << std::endl;


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
	    ss << "Custom values->" << lower << " " << upper;
	    break;

	    case 2:
	    ss << "Tracking hand";
	    break;

	}
	ss << " Angle: " << (int) handAngle << "";
	std::string text = ss.str();
	cv::putText( display, text.c_str(), cv::Point(0, 18),
		     cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(0, 0, 255));

	//-----------------------------------------------------------------------------------------------------
	//-- Move cursor
	//-----------------------------------------------------------------------------------------------------
	if ( debugValue == 2)
	{
	    if ( (int) handContour.size() > 0 )
	    {
		//-- Get image dimensions:
		int imageWidth = frame.cols, imageHeight = frame.rows;
		//std::cout << "Captured image: " << imageWidth << " x " << imageHeight << std::endl;

		//-- Get hand center (uncomment the selected method):
		int cogX = 0, cogY = 0;

		//-------------------- With RotatedRect --------------------------------------
		/*
		cv::RotatedRect minRect = cv::minAreaRect( handContour[0]);
		cv::Point2f rect_points[4]; minRect.points( rect_points );
		for( int j = 0; j < 4; j++ )
		{
		    cogX += rect_points[j].x;
		    cogY += rect_points[j].y;
		}

		cogX /= 4;
		cogY /= 4;
		*/

		//-------------------- With Bounding Rectangle --------------------------------
		//-- (This is more stable than the rotated rectangle)
		cv::Rect rect  =  cv::boundingRect(handContour[0]);
		cogX = rect.x + rect.width / 2;
		cogY = rect.y + rect.height / 2;


		//-- (Optional) Print cog on screen:
		cv::circle( display, cv::Point(cogX, cogY), 5, cv::Scalar( 255, 0, 0), 2 );

		//-- Get screen dimensions:
		int screenHeight, screenWidth;
		getDisplayDimensions( screenWidth, screenHeight);

		//-- Get new cursor position by mapping the points:
		int x, y;
		x = cogX * screenWidth / imageWidth;
		y = cogY * screenHeight / imageHeight;

		//-- Move the mouse to the specified position:
		moveMouse( x, y );
	    }
	}

	//-----------------------------------------------------------------------------------------------------
	//-- Show processed image
	//-----------------------------------------------------------------------------------------------------
	cv::imshow( "Processed Stream", display);


	//-----------------------------------------------------------------------------------------------------
	//-- Print "Angle control"
	//-----------------------------------------------------------------------------------------------------
	cv::Mat gauge = cv::Mat::zeros( 100, 200, CV_8UC3);

	int gauge_l = 60;
	double rad_ang = handAngle * 3.1415 / 180.0;
	int x_coord = gauge_l * cos( rad_ang);
	int y_coord = gauge_l * sin( rad_ang);

	cv::line( gauge, cv::Point( 200/2, 80 ), cv::Point( 200/2 + x_coord , 80 - y_coord), cv::Scalar( 0, 0, 255));
	cv::imshow( "Gauge", gauge);

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


