#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../libraries/HandDetector.h"

void processFrame(cv::Mat &src, cv::Mat &dst, int debug  = 0);
void drawCalibrationMarks(cv::Mat &input, int halfSide = 20,  cv::Scalar color  = cv::Scalar( 0, 255, 0 ) );
int getSkinHueValue( cv::Mat &hue_image, int& result,  int halfSide = 20 );
void getHue( cv::Mat& src, cv::Mat& hue);

int main( int argc, char * argv[] )
{
    //-- Setup video
    //--------------------------------------------------------------

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
    delay = 24; //-- Force 24 FPS


    //-- Declare variables
    //--------------------------------------------------------------------
    bool stop = false;
    int debugValue = 0;

    //-- For getting skin hue value:
    int skinValue;
    static const int halfSide = 40;

   // cv::BackgroundSubtractorMOG2 substractor;
   // substractor.bShadowDetection = false;

    //-- Calibration loop
    //--------------------------------------------------------------------
    cv::namedWindow( "Calibrating skin", cv::WINDOW_AUTOSIZE);

    while( !stop)
    {
	//-- Get current frame
	cv::Mat frame;
	if (! cap.read( frame ) )
	    break;

	//-- Add calibration frame
	drawCalibrationMarks(frame, halfSide);

	//-- Show calibration screen
	cv::imshow( "Calibrating skin", frame );

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
	    getSkinHueValue( ROI, skinValue);
	    std::cout << "Skin average value is: " << skinValue << std::endl;
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
		debugValue = -1;
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


void processFrame( cv::Mat& src, cv::Mat& dst, int debug)
{

    if (debug == 0 )
    {
	//canny( src, dst);
	//substractor(src, dst);

	//-- Obtain hue channel:
	cv::Mat hue;
	getHue( src, hue);

	cv::Mat thresh;
	cv::threshold( hue, thresh, 45, 255, cv::THRESH_BINARY);
	//cv::inRange(hsv, Scalar(0, 58, 89), Scalar(25, 173, 229), bw);


	dst = thresh;
    }
    else if (debug == -1)
    {
	//-- Do nothing (is this possible?)
	return;
    }

}

void drawCalibrationMarks( cv::Mat& input, int halfSide, cv::Scalar color)
{
    //-- Get image dimensions
    int image_cols = input.cols;
    int image_rows = input.rows;

    //-- Draw a rectangle in the output image
    cv::rectangle( input,
		   cv::Point( image_cols / 2 - halfSide,  image_rows/2 - halfSide ),
		   cv::Point( image_cols / 2 + halfSide,  image_rows/2 + halfSide),
		   color
		   );
}

void getHue( cv::Mat& src, cv::Mat& hue)
{
    //-- Convert the image to HSV
    cv::Mat dst;
    cv::cvtColor( src, dst, CV_BGR2HSV);

    //-- Extract hue channel
    std::vector<cv::Mat> hsv;
    cv::split( dst, hsv);
    hue = hsv[0];

}

int getSkinHueValue( cv::Mat& ROI , int& result,  int halfSide )
{
    //-- Obtain hue channel:
    cv::Mat hue;
    getHue( ROI, hue);

    //-- Calculate average value
    int average = 0;

    for (int i = 0; i < hue.cols; i++ )
	for( int j = 0; j < hue.rows; j++)
	{   std::cout << hue.at<unsigned int>(i, j) << std::endl;
	    average += hue.at<unsigned int>(i, j);
}
    average /= (hue.cols * hue.rows);

    result = average;
}
