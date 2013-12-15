#ifndef BACKGROUNDSUBSTRACTOR_H
#define BACKGROUNDSUBSTRACTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class backgroundSubstractor: public cv::BackgroundSubtractorMOG2
{
public:
    void setbackgroundRatio(float a){backgroundRatio = a;}
};

#endif // BACKGROUNDSUBSTRACTOR_H
