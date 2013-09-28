#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../libraries/HandDetector.h"
#include "../libraries/handUtils.h"


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
	cv::Mat processed = frame.clone();
	switch( debugValue )
	{
	    case 0:
		handDetector.calibrate();
		handDetector( frame, processed);
		cv::putText( processed, "Default values", cv::Point(0, 18),
			     cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(0, 0, 255));
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


	//-- Show processed image
	cv::imshow( "Processed Stream", processed);


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




void processFrame( cv::Mat& src, cv::Mat& dst, int hueThValue, int hueRangeValue, int satThValue, int valThLower, int valThUpper, int debug)
{

    if (debug == 0 || debug == 4)
    {
	//canny( src, dst);

	//-- Get hand thresholded:
	cv::Mat thresholdedHand;
	if (debug == 0)
	{
	    getThresholdedHand( src, thresholdedHand, hueThValue, hueRangeValue, satThValue, valThLower, valThUpper);
	}
	else
	{
	    //-- Obtain HSV channels:
	    //------------------------------------------------------------------------------------------------
	    cv::Mat hsv;
	    cv::cvtColor( src, hsv, CV_BGR2HSV);

	    //-- Hardcoded skin value:
	    cv::inRange(hsv, cv::Scalar(0, 58, 89), cv::Scalar(25, 173, 229), thresholdedHand);
	}

	//-- Filter out blobs:
	cv::Mat blobsFiltered;
	cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 5, 5) );
	cv::morphologyEx( thresholdedHand, blobsFiltered, cv::MORPH_CLOSE, kernel);

	//-- Find contours:
	std::vector< std::vector< cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours( blobsFiltered, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0) );

	//-- Find largest contour:
	int largestId = 0, largestValue = 0;
	for (int i = 0; i < contours.size(); i++)
	    if ( contours[i].size() > largestValue )
	    {
		largestId = i;
		largestValue = contours[i].size();
	    }

	//-- Draw contours:
	//dst = cv::Mat::zeros( blobsFiltered.size(), CV_8UC3);
	cv::cvtColor( blobsFiltered, dst, CV_GRAY2BGR );
	//for (int i = 0; i < contours.size(); i++)
	    cv::drawContours( dst, contours, largestId, cv::Scalar( 0, 0, 255), 1, 8);
	    cv::fillConvexPoly( dst, contours[largestId], cv::Scalar( 255, 255, 255));

	//-- Draw rotated rectangle:
	cv::RotatedRect minRect = cv::minAreaRect( contours[largestId]);
	cv::Point2f rect_points[4]; minRect.points( rect_points );
	for( int j = 0; j < 4; j++ )
	    cv::line( dst, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255, 0, 0) , 1, 8 );

	//-- Bounding rectangle:
	//cv::rectangle( dst, cv::boundingRect( contours[largestId]), cv::Scalar(255, 0, 0) , 1, 8 );
    }

    else if (debug == -1)
    {
	//-- Do nothing
	return;
    }
    else if (debug == 2)
    {

	//-- Obtain HSV channels:
	//------------------------------------------------------------------------------------------------
	cv::Mat hsv;
	cv::cvtColor( src, hsv, CV_BGR2HSV);

	//-- Hardcoded skin value:
	cv::inRange(hsv, cv::Scalar(0, 58, 89), cv::Scalar(25, 173, 229), dst);
    }
    else if (debug == 3 )
    {
	//-- Get hand thresholded:
	getThresholdedHand( src, dst, hueThValue, hueRangeValue, satThValue, valThLower, valThUpper);
    }
}


void getThresholdedHand(cv::Mat& src, cv::Mat& dst, int hueThValue, int hueRangeValue, int satThValue, int valThLower, int valThUpper )
{
    //-- Obtain HSV channels:
    //------------------------------------------------------------------------------------------------
    cv::Mat srcHSV;
    cv::cvtColor( src, srcHSV, CV_BGR2HSV);

    std::vector<cv::Mat> hsv;
    cv::split( srcHSV, hsv);
    cv::Mat hue = hsv[0], sat = hsv[1], val = hsv[2];


    //-- Threshold channels:
    //------------------------------------------------------------------------------------------------
    cv::Mat hueThresh, satThresh, valThresh;

    //-- Hue
    filterHueRange( hue, hueThresh, hueThValue, hueRangeValue);
    //cv::imshow( "Hue", hueThresh);

    //-- Saturation
    cv::threshold( sat, satThresh, satThValue, 255, cv::THRESH_BINARY);
    //cv::imshow( "Sat", satThresh);

    //-- Value
    if (valThUpper == -1 )
    {
	cv::threshold( val, valThresh, valThLower, 255, cv::THRESH_BINARY);
    }
    else
    {
	cv::inRange( val, valThLower, valThUpper, valThresh);
    }
    //cv::imshow( "Val", valThresh);

    //-- Apply an 'and' operation to the three thresholds
    cv::bitwise_and( satThresh, valThresh, dst);
    cv::bitwise_and( hueThresh, dst, dst);
}


void drawCalibrationMarks( cv::Mat& input, cv::Mat& output, int halfSide, cv::Scalar color)
{
    //-- Get image dimensions
    int image_cols = input.cols;
    int image_rows = input.rows;

    //-- Draw a rectangle in the output image
    output = input.clone();
    cv::rectangle( output,
		   cv::Point( image_cols / 2 - halfSide,  image_rows/2 - halfSide ),
		   cv::Point( image_cols / 2 + halfSide,  image_rows/2 + halfSide),
		   color
		   );
}



