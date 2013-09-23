#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../libraries/HandDetector.h"

void processFrame(cv::Mat &src, cv::Mat &dst, int& thresholdValue, int debug  = 0);
void drawCalibrationMarks(cv::Mat& input, cv::Mat& output, int halfSide = 20,  cv::Scalar color  = cv::Scalar( 0, 255, 0 ) );
int getSkinHueValue( cv::Mat &hue_image, int& result);
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
	cv::flip(frame,frame,1);

	//-- Process it
	cv::Mat processed = frame.clone();
	processFrame( frame, processed, skinValue, debugValue);

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


void processFrame( cv::Mat& src, cv::Mat& dst, int& thresholdValue, int debug)
{

    if (debug == 0 )
    {
	//canny( src, dst);
	//substractor(src, dst);


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
	int interval = 10;
	cv::inRange( hue, thresholdValue-interval/2 < 0 ? 180-thresholdValue : thresholdValue-interval/2,
			      thresholdValue+interval/2 > 180 ? thresholdValue : thresholdValue+interval/2,
			      hueThresh);

	cv::imshow( "Hue", hueThresh);

	//-- Saturation
	cv::threshold( sat, satThresh, 30, 0, cv::THRESH_TOZERO);
	cv::imshow( "Sat", satThresh);

	//-- Value
	cv::threshold( val, valThresh, 60, 0, cv::THRESH_TOZERO);
	cv::imshow( "Val", valThresh);


	//-- Joint channels and convert back to BGR
	//------------------------------------------------------------------------------------------------
	std::vector<cv::Mat> hsvThresh;
	hsvThresh.push_back(hueThresh);
	hsvThresh.push_back(satThresh);
	hsvThresh.push_back(valThresh);

	cv::merge( hsvThresh, dst);
	cv::cvtColor( dst, dst, CV_HSV2BGR);

	//dst = satThresh;
    }
    else if (debug == -1)
    {
	//-- Do nothing
	return;
    }

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

int getSkinHueValue( cv::Mat& ROI , int& result )
{
    //-- Obtain hue channel:
    cv::Mat hue;
    getHue( ROI, hue);

    //-- Calculate average value
    int average = 0;

    for (int i = 0; i < hue.cols; i++ )
	for( int j = 0; j < hue.rows; j++)
	    average += (int) hue.at<unsigned char>(i, j);

    average /= (hue.cols * hue.rows);
    result = average;
}
