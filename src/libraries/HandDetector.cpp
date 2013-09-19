#include "HandDetector.h"

HandDetector::HandDetector(const std::string& name)
{
     capture.open(name);
}

HandDetector::HandDetector( const int& device)
{
    capture.open(device);
}

void HandDetector::setFunction( void (*function)(cv::Mat&, cv::Mat&) )
{
    this->function = function;
}

