#ifndef HANDUTILS_H
#define HANDUTILS_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "backgroundSubstractor.h"

void drawCalibrationMarks(cv::Mat& input, cv::Mat& output, int halfSide = 20,  cv::Scalar color  = cv::Scalar( 0, 255, 0 ) );

void drawHistogram(const cv::Mat& image);
void drawHistogramHSV( const cv::Mat& image);

//-- Contour finding and filtering:
void getContours(const cv::Mat &src, std::vector<std::vector<cv::Point> > &contours);
void filterContours(std::vector< std::vector<cv::Point> >& srcContours, std::vector< std::vector<cv::Point> >& handContour , const int min = 250, const int max = 1800);

//-- Rectangle characterization:
double getAngle( cv::RotatedRect boundingRect);

//-- Static background substractor
void backgroundSubs(cv::Mat &, backgroundSubstractor & bg);

//-- More useful way of storing convexity defects:
typedef struct
{
    cv::Point start;
    cv::Point end;
    cv::Point depth_point;
    double depth;

    int start_index;
    int end_index;
    int depth_point_index;
}  ConvexityDefect;

//-- Find the angle between two lines (using three points)
//-- Note:
//-- The angle obtained here is the angle going from start-vertex segment to end-vertex segment
float findAngle( cv::Point start, cv::Point end, cv::Point vertex);


//-- Prints a progress bar on the bottom of the screen
void printProgressBar( cv::Mat& src, cv::Mat& dst, float percentage, cv::Scalar color, int thickness = 25 );
#endif // HANDUTILS_H
