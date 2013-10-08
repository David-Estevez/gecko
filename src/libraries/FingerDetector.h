#ifndef FINGERDETECTOR_H
#define FINGERDETECTOR_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include<algorithm>

class FingerDetector 
{
	public:
		//-- Constructors
		FingerDetector();
		FingerDetector(cv::Mat &, int &);
		int getFingers();
		
	private:	
	
		//-- Geometric functions
		double distance(cv::Point,cv::Point );
		std::pair<cv::Point,double> circle(cv::Point, cv::Point, cv::Point);
		int number_of_fingers; 
		
};
 
 
 
#endif // FINGERDETECTOR_H
