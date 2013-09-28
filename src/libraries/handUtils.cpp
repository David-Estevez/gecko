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

void drawHistogram(const cv::Mat& img)
{
    int bins = 256;
    int nc = img.channels();
    std::vector<cv::Mat> hist( nc);
    std::vector<cv::Mat> canvas( nc);
    int hmax[3] = {0,0,0};

    for (int i = 0; i < hist.size(); i++)
	hist[i] = cv::Mat::zeros(1, bins, CV_32SC1);

    for (int i = 0; i < img.rows; i++)
    {
	for (int j = 0; j < img.cols; j++)
	{
	    for (int k = 0; k < nc; k++)
	    {
		uchar val = nc == 1 ? img.at<uchar>(i,j) : img.at<cv::Vec3b>(i,j)[k];
		hist[k].at<int>(val) += 1;
	    }
	}
    }

    for (int i = 0; i < nc; i++)
    {
	for (int j = 0; j < bins-1; j++)
	    hmax[i] = hist[i].at<int>(j) > hmax[i] ? hist[i].at<int>(j) : hmax[i];
    }

    const char* wname[3] = { "channel 1", "channel 2", "channel 3" };
    cv::Scalar colors[3] = { cv::Scalar(255,0,0), cv::Scalar(0,255,0), cv::Scalar(0,0,255) };

    for (int i = 0; i < nc; i++)
    {
	canvas[i] = cv::Mat::ones(125, bins, CV_8UC3);

	for (int j = 0, rows = canvas[i].rows; j < bins-1; j++)
	{
	    cv::line(
			canvas[i],
			cv::Point(j, rows),
			cv::Point(j, rows - (hist[i].at<int>(j) * rows/hmax[i])),
			nc == 1 ? cv::Scalar(200,200,200) : colors[i],
			1, 8, 0
			);
	}

	cv::imshow(nc == 1 ? "value" : wname[i], canvas[i]);
    }
}

void drawHistogramHSV(const cv::Mat &image)
{
    cv::Mat image_hsv;
    cv::cvtColor( image, image_hsv, CV_BGR2HSV);

    drawHistogram( image_hsv);
}
