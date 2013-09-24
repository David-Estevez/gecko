#ifndef HANDUTILS_H
#define HANDUTILS_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void processFrame( cv::Mat& src, cv::Mat& dst, int hueThValue, int hueRangeValue, int satThValue = 30, int valThLower = 60, int valThUpper = -1, int debug = 0);
void drawCalibrationMarks(cv::Mat& input, cv::Mat& output, int halfSide = 20,  cv::Scalar color  = cv::Scalar( 0, 255, 0 ) );
void getThresholdedHand(cv::Mat& src, cv::Mat& dst, int hueThValue, int hueRangeValue, int satThValue, int valThLower, int valThUpper );

void getHue( cv::Mat& src, cv::Mat& hue);
void filterHueRange( cv::Mat& hueSrc, cv::Mat& dst, int hueValue, int range);

double getAverage( cv::Mat &ROI);
double getStdDev( cv::Mat& ROI );

#endif // HANDUTILS_H
