#include "HandDetector.h"

//--------------------------------------------------------------------------------------------------------
//-- Constructors
//--------------------------------------------------------------------------------------------------------

HandDetector::HandDetector()
{
    //-- Std deviation multipliers:
    hue_sigma_mult = 6;
    sat_sigma_mult = 6;
    val_sigma_mult = 6;

    //-- Skin color limits
    lower_limit = cv::Scalar( 0, 58, 89);
    upper_limit = cv::Scalar( 25, 173, 229);
    hue_invert = false;

}

HandDetector::HandDetector( cv::Mat& ROI)
{
    //-- Std deviation multipliers:
    hue_sigma_mult = 2;
    sat_sigma_mult = 2;
    val_sigma_mult = 2;

    //-- Skin color limits
    calibrate( ROI );
}


//--------------------------------------------------------------------------------------------------------
//-- Calibration functions
//--------------------------------------------------------------------------------------------------------

void HandDetector::calibrate(cv::Mat &ROI)
{
    //-- Convert from BGR to HSV
    cv::Mat HSV_ROI;
    cv::cvtColor( ROI, HSV_ROI, CV_BGR2HSV);

    //-- Split HSV ROI in channels:
    std::vector<cv::Mat> HSV_splitted;
    cv::split( HSV_ROI, HSV_splitted);

    //-- Calculate values:
    int hue_mean = median( HSV_splitted[0] );
    //int sat_mean = median( HSV_splitted[1] );
    //int val_mean = median( HSV_splitted[2] );

    int hue_sigma = stdDeviation( HSV_splitted[0] );
    //int sat_sigma = stdDeviation( HSV_splitted[1] );
    //int val_sigma = stdDeviation( HSV_splitted[2] );

    //-- Calculate limits
    int hue_lower_limit = hue_mean - hue_sigma_mult * hue_sigma / 2;
    //int sat_lower_limit = sat_mean - sat_sigma_mult * sat_sigma / 2;
    //int val_lower_limit = val_mean - val_sigma_mult * val_sigma / 2;

    int hue_upper_limit = hue_mean + hue_sigma_mult * hue_sigma / 2;
    //int sat_upper_limit = sat_mean + sat_sigma_mult * sat_sigma / 2;
    //int val_upper_limit = val_mean + val_sigma_mult * val_sigma / 2;

    //-- Check the limits of the values
    if ( hue_lower_limit < 0)
    {
	int aux = hue_lower_limit;
	hue_lower_limit = hue_upper_limit;
	hue_upper_limit = 180 - abs(aux);
	hue_invert = true;
    }
    else if ( hue_upper_limit > 179)
    {
	int aux = hue_upper_limit;
	hue_upper_limit = hue_lower_limit;
	hue_lower_limit = aux - 180;
	hue_invert = true;
    }
    else
	hue_invert = false;
/*
    if ( sat_lower_limit < 0) sat_lower_limit = 0;
    if ( val_lower_limit < 0) val_lower_limit = 0;
    if ( sat_upper_limit > 255 ) sat_upper_limit = 255;
    if ( val_lower_limit > 255 ) val_upper_limit = 255;
*/
    //-- Compose the values:
  //  lower_limit = cv::Scalar( hue_lower_limit, sat_lower_limit, val_lower_limit );
  //  upper_limit = cv::Scalar( hue_upper_limit, sat_upper_limit, val_upper_limit );

    lower_limit = cv::Scalar( hue_lower_limit, 58, 89  );
    upper_limit = cv::Scalar( hue_upper_limit, 173, 229 );

    std::cout << "[Debug] Lower limit is: " << lower_limit << std::endl;
    std::cout << "[Debug] Upper limit is: " << upper_limit << std::endl;
    std::cout << "[Debug] Inverting hue: " << hue_invert << std::endl;
}


void HandDetector::calibrate(cv::Scalar lower_limit, cv::Scalar upper_limit)
{
    this->lower_limit = lower_limit;
    this->upper_limit = upper_limit;
    this->hue_invert = false;
}

void HandDetector::getCalibration(cv::Scalar &lower_limit, cv::Scalar &upper_limit)
{
    lower_limit = this->lower_limit;
    upper_limit = this->upper_limit;
}

//--------------------------------------------------------------------------------------------------------
//-- Hand-detection
//--------------------------------------------------------------------------------------------------------

void HandDetector::operator ()( const cv::Mat& src, cv::Mat& dst)
{
    filter_hand( src, dst);
}


void HandDetector::filter_hand(const cv::Mat &src, cv::Mat &dst)
{
    //-- HSV thresholding
    //----------------------------------------------------------------------------------------------------
    //-- Convert to HSV
    cv::Mat hsv;
    cv::cvtColor( src, hsv, CV_BGR2HSV);

    //-- Threshold
    cv::Mat thresholdedHand;
    cv::inRange(hsv, lower_limit, upper_limit, thresholdedHand);

    //-- If color limit is arround 0, hue channel needs to be inverted
    if (hue_invert)
    {
		std::vector< cv::Mat > hsv;
		cv::split( thresholdedHand, hsv);
		cv::bitwise_not( hsv[0], hsv[0] );
    }

    //-- Filtering
    //----------------------------------------------------------------------------------------------------
    //-- Filter out blobs:
    cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 5, 5) );
    cv::morphologyEx( thresholdedHand, dst, cv::MORPH_CLOSE, kernel);

}


//--------------------------------------------------------------------------------------------------------
//-- Statistical functions:
//--------------------------------------------------------------------------------------------------------

int HandDetector::average(cv::Mat &ROI)
{
    //-- Calculate average value
    double average = 0;

    for (int i = 0; i < ROI.cols; i++ )
	for( int j = 0; j < ROI.rows; j++)
	    average += (int) ROI.at<unsigned char>(i, j);


    average /= (double) (ROI.cols * ROI.rows);
    return ceil(average);
}

int HandDetector::stdDeviation(cv::Mat &ROI)
{
    //-- Calculates the std deviation of the pixel values
    double stddev = 0;

    for (int i = 0; i < ROI.cols; i++ )
	for( int j = 0; j < ROI.rows; j++)
	    stddev +=  pow( (int) ROI.at<unsigned char>(i, j), 2);

    stddev = sqrt( stddev ) / (ROI.cols * ROI.rows);
    return ceil(stddev);
}

int HandDetector::median(cv::Mat &ROI)
{
    //-- Unroll the ROI
    std::vector<unsigned char> unrolled;

    for (int i = 0; i < ROI.cols; i++ )
	for( int j = 0; j < ROI.rows; j++)
	     unrolled.push_back( ROI.at<unsigned char>(i, j) );

    //-- Sort the vector
    std::sort( unrolled.begin(), unrolled.end() );

    //-- Get the median
    if ( unrolled.size() % 2 != 0)
    {
	//-- Odd vector:
	return ceil( unrolled.at( (unrolled.size() -1 ) / 2) );
    }
    else
    {
	//-- Even vector:
	double mean = unrolled.at( unrolled.size() / 2 ) + unrolled.at( (unrolled.size() / 2 ) - 1 );
	mean /= 2;
	return ceil(mean);
    }
}




