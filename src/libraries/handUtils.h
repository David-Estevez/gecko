#ifndef HANDUTILS_H
#define HANDUTILS_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void drawCalibrationMarks(cv::Mat& input, cv::Mat& output, int halfSide = 20,  cv::Scalar color  = cv::Scalar( 0, 255, 0 ) );

void drawHistogram(const cv::Mat& image);
void drawHistogramHSV( const cv::Mat& image);

//-- Contour finding and filtering:
void getContours(const cv::Mat &src, std::vector<std::vector<cv::Point> > &contours);
void filteredContour(cv::Mat& src, std::vector< std::vector<cv::Point> >  &handContour , const int min = 350, const int max = 1800);

//-- Rectangle characterization:
double getAngle( cv::RotatedRect boundingRect);

//-- Static background substractor

cv:: Mat sBackgroundSubs(cv::Mat );	

#endif // HANDUTILS_H