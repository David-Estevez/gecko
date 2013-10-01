#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../libraries/HandDetector.h"
#include "../libraries/handUtils.h"

void processHand( cv::Mat& src, cv::Mat& dst, cv::Mat& handMask);

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
    bool stop = false;
    int debugValue = 0;

    //-- These are the new, compact variables
    HandDetector handDetector;
    cv::Scalar lower, upper;
    static const int halfSide = 40;



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
	    cv::imshow( "Test", ROI);

	    handDetector.calibrate( ROI );
	    handDetector.getCalibration( lower, upper);

	    drawHistogramHSV( ROI );

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
	//-- Get current frame
	cv::Mat frame;
	if ( ! cap.read( frame ) )
	    break;
	cv::flip(frame,frame,1);

	//-- Process it
	cv::Mat processed;
	switch( debugValue )
	{
	    case 0:
		handDetector.calibrate();
		handDetector( frame, processed);
		break;

	    case 1:
		handDetector.calibrate( lower, upper);
		handDetector( frame, processed);
		/*
		std::stringstream message;
		message << "Custom values: " << lower << ", " << upper ;
		std::string mssg; message >> mssg;
		cv::putText( processed, mssg.c_str(), cv::Point(0, 18),
			     cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(0, 0, 255));
		*/
		break;

	    default:
		break;
	   }

	//-- Show it:
	cv::Mat display;
	processHand( frame, display, processed);

	//-- Adding text:
	switch( debugValue)
	{
	    case 0:
	    cv::putText( display, "Mode: Default values", cv::Point(0, 18),
			 cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(0, 0, 255));
	    break;

	    case 1:
	    cv::putText( display, "Mode: Custom values", cv::Point(0, 18),
		     cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(0, 0, 255));
	    break;

	}


	//-- Show processed image
	cv::imshow( "Processed Stream", display);


	//-- Decide what to do next depending on key pressed
	char key = (char) cv::waitKey( delay);
	switch( key)
	{
	    case 'f': //-- Filtered frame
		debugValue = 0;
		break;
	    case 'b': //-- By-pass filters
		debugValue = -1;
		break;
	    case 'd': //-- hardcoded filter
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

void processHand( cv::Mat& src, cv::Mat& dst, cv::Mat& handMask)
{
    //-- Obtain contours:
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(handMask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0) );

    //-- Filter contours:
    std::vector<std::vector<cv::Point> > filteredContours;
    const int min = 500, max = 1400;

    for (int i = 0; i < contours.size(); i++)
	if ( (int) contours[i].size() > min && (int) contours[i].size() < max )
	    filteredContours.push_back( contours[i] );

    //std::cout << "[Debug] Contours before: " << contours.size() << " Contours after: " << filteredContours.size() << std::endl;


    //-- Find largest contour:
    int largestId = 0, largestValue = 0;
    for (int i = 0; i < contours.size(); i++)
	if ( (int) contours[i].size() > largestValue )
	{
	    largestId = i;
	    largestValue = (int) contours[i].size();
	}
    //std::cout << "[Debug] Number of contours: " << (int) contours.size() << " Largest contour: " << (int) largestValue << std::endl;

    //-- Draw things:
    const bool displayContour = true;
    const bool displayBoundingBox = false;
    const bool displayBoundingRotRect = true;

    if ( dst.total() == 0)
	dst = src.clone();

    if ( contours.size() > 0 )
    {
	//-- Draw contours:
	if (displayContour  )
	{
	    cv::drawContours( dst, contours, largestId, cv::Scalar( 0, 0, 255), 1, 8);
	    //cv::fillConvexPoly( dst, contours[largestId], cv::Scalar( 255, 255, 255));
	}

	//-- Draw rotated rectangle:
	if ( displayBoundingRotRect )
	{
	    cv::RotatedRect minRect = cv::minAreaRect( contours[largestId]);
	    cv::Point2f rect_points[4]; minRect.points( rect_points );
	    for( int j = 0; j < 4; j++ )
		cv::line( dst, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255, 0, 0) , 1, 8 );

	    std::cout << "Box Angle: " << getAngle(minRect) << std::endl;
    	}

	//-- Bounding rectangle:
	if (displayBoundingBox)
	    cv::rectangle( dst, cv::boundingRect( contours[largestId]), cv::Scalar(255, 0, 0) , 1, 8 );
    }
    else
	std::cerr << "No contours found!" << std::endl;
}



