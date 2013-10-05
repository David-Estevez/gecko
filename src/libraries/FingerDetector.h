#ifndef FINGERDETECTOR_H
#define FINGERDETECTOR_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

class FingerDetector 
{
	public:
		//-- Constructors
		FingerDetector();
		FingerDetector(cv::Mat &, int &);
		
};
 
 
 
#endif // FINGERDETECTOR_H
