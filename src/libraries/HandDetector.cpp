#include "HandDetector.h"

//-- Constructors

HandDetector::HandDetector()
{
    //-- Std deviation multipliers:
    hue_sigma_mult = 2;
    sat_sigma_mult = 2;
    val_sigma_mult = 2;

    //-- Skin color limits
    lower_limit = cv::Scalar( 0, 58, 89);
    upper_limit = cv::Scalar( 25, 173, 229);

    //-- Display options:
    displayBoundingBox = false;
    displayBoundingRotRect = true;
    displayContour = true;
    displayThreshold = true;
}

HandDetector::HandDetector( cv::Mat& ROI)
{
    //-- Std deviation multipliers:
    hue_sigma_mult = 2;
    sat_sigma_mult = 2;
    val_sigma_mult = 2;

    //-- Skin color limits
    calibrate( ROI );

    //-- Display options:
    displayBoundingBox = false;
    displayBoundingRotRect = true;
    displayContour = true;
    displayThreshold = true;
}


//-- Calibration functions

void HandDetector::calibrate(cv::Mat &ROI)
{
    //-- Convert from BGR to HSV
    cv::Mat HSV_ROI;
    cv::cvtColor( ROI, HSV_ROI, CV_BGR2HSV);

    //-- Split HSV ROI in channels:
    std::vector<cv::Mat> HSV_splitted;
    cv::split( HSV_ROI, HSV_splitted);

    //-- Calculate values:
    int hue_mean = average( HSV_splitted[0] );
    int sat_mean = average( HSV_splitted[1] );
    int val_mean = average( HSV_splitted[2] );

    int hue_sigma = stdDeviation( HSV_splitted[0] );
    int sat_sigma = stdDeviation( HSV_splitted[1] );
    int val_sigma = stdDeviation( HSV_splitted[2] );

    //-- Compose the values:
    lower_limit = cv::Scalar( hue_mean - hue_sigma_mult * hue_sigma / 2,
			      sat_mean - sat_sigma_mult * sat_sigma / 2,
			      val_mean - val_sigma_mult * val_sigma / 2 );

    upper_limit = cv::Scalar( hue_mean + hue_sigma_mult * hue_sigma / 2,
			      sat_mean + sat_sigma_mult * sat_sigma / 2,
			      val_mean + val_sigma_mult * val_sigma / 2);


}


void HandDetector::calibrate(cv::Scalar lower_limit, cv::Scalar upper_limit)
{
    this->lower_limit = lower_limit;
    this->upper_limit = upper_limit;
}



//-- Hand-detection

void HandDetector::operator ()( const cv::Mat& src, cv::Mat& dst)
{
    find( src, dst);
}


void HandDetector::find(const cv::Mat &src, cv::Mat &dst)
{
    //-- HSV thresholding
    //----------------------------------------------------------------------------------------------------
    //-- Convert to HSV
    cv::Mat hsv;
    cv::cvtColor( src, hsv, CV_BGR2HSV);

    //-- Threshold
    cv::Mat thresholdedHand;
    cv::inRange(hsv, lower_limit, upper_limit, thresholdedHand);

    //-- Supposedly, this function was needed to filter values of red around the 0 hue value
    //-- It needs to be adapted for the class version
    //getThresholdedHand( src, thresholdedHand, hueThValue, hueRangeValue, satThValue, valThLower, valThUpper);


    //-- Filtering
    //----------------------------------------------------------------------------------------------------
    //-- Filter out blobs:
    cv::Mat blobsFiltered;
    cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 5, 5) );
    cv::morphologyEx( thresholdedHand, blobsFiltered, cv::MORPH_CLOSE, kernel);


    //-- Contours:
    //----------------------------------------------------------------------------------------------------
    //-- Find contours:
    std::vector< std::vector< cv::Point> > contours;
    cv::findContours( blobsFiltered, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0) );

    //-- Find largest contour:
    int largestId = 0, largestValue = 0;
    for (int i = 0; i < contours.size(); i++)
	if ( contours[i].size() > largestValue )
	{
	    largestId = i;
	    largestValue = contours[i].size();
	}

    //-- Display things:
    //---------------------------------------------------------------------------------------------------
    if ( displayThreshold && ( displayBoundingBox || displayBoundingRotRect || displayContour) )
    {
	cv::cvtColor( blobsFiltered, dst, CV_GRAY2BGR );

	//-- Draw contours:
	if (displayContour)
	{
	    cv::drawContours( dst, contours, largestId, cv::Scalar( 0, 0, 255), 1, 8);
	    cv::fillConvexPoly( dst, contours[largestId], cv::Scalar( 255, 255, 255));
	}

	//-- Draw rotated rectangle:
	if ( displayBoundingRotRect )
	{
	    cv::RotatedRect minRect = cv::minAreaRect( contours[largestId]);
	    cv::Point2f rect_points[4]; minRect.points( rect_points );
	    for( int j = 0; j < 4; j++ )
		cv::line( dst, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255, 0, 0) , 1, 8 );
	}

	//-- Bounding rectangle:
	if (displayBoundingBox)
	    cv::rectangle( dst, cv::boundingRect( contours[largestId]), cv::Scalar(255, 0, 0) , 1, 8 );
    }

}




//-- Display configuration

void HandDetector::setDisplayBoundingBox( const bool displayBoundingBox)
{
    this->displayBoundingBox = displayBoundingBox;
}

void HandDetector::setDisplayBoundingRotRect( const bool displayBoundingRotRect)
{
    this->displayBoundingRotRect = displayBoundingRotRect;
}

void HandDetector::setDisplayContour( const bool displayContour)
{
    this->displayContour = displayContour;
}

void HandDetector::setDisplayThreshold( const bool displayThreshold)
{
    this->displayThreshold = displayThreshold;
}

void HandDetector::setDisplay( const bool displayBoundingBox, const bool displayBoundingRotRect, const bool displayContour, const bool displayThreshold)
{
    this->displayBoundingBox = displayBoundingBox;
    this->displayBoundingRotRect = displayBoundingRotRect;
    this->displayContour = displayContour;
    this->displayThreshold = displayThreshold;
}



//-- Statistical functions:

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
