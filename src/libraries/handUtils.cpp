#include "handUtils.h"

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

double getAverage( cv::Mat& ROI )
{
    //-- Obtain hue channel:
    cv::Mat hue;
    getHue( ROI, hue);

    //-- Calculate average value
    int average = 0;

    for (int i = 0; i < hue.cols; i++ )
	for( int j = 0; j < hue.rows; j++)
	    average += (int) hue.at<unsigned char>(i, j);


    average /= (double) (hue.cols * hue.rows);
    return average;
}

double getStdDev( cv::Mat& ROI )
{
    //-- Calculates the std deviation of the pixel values
    double stddev = 0;

    for (int i = 0; i < ROI.cols; i++ )
	for( int j = 0; j < ROI.rows; j++)
	    stddev +=  pow( (int) ROI.at<unsigned char>(i, j), 2);

    stddev = sqrt( stddev ) / (ROI.cols * ROI.rows);
    std::cout << "Std deviation: " << stddev << std::endl;
    return stddev;
}

void filterHueRange( cv::Mat& hueSrc, cv::Mat& dst, int hueValue, int range)
{
    int lowerBound = hueValue - range/2, upperBound = hueValue + range/2;

    if (  lowerBound  >= 0 && upperBound < 180 )
    {
	//-- Usual range
	cv::inRange( hueSrc, lowerBound, upperBound, dst );
    }
    else
    {

	if (lowerBound < 0 )  	//-- Close to red (0)
	{
	    int aux = lowerBound;
	    lowerBound = upperBound;
	    upperBound = 180 - abs(aux);
	}
	else if ( upperBound > 179 )  //-- Close to red (179)
	{
	    int aux = upperBound;
	    upperBound = lowerBound;
	    lowerBound = aux - 180;
	}

	std::cout << "Value limits: " << lowerBound << " to " << upperBound << std::endl;
	cv::inRange( hueSrc, lowerBound, upperBound, dst );
	cv::bitwise_not( dst, dst);

    }
}

