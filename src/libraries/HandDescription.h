#ifndef HAND_DESCRIPTION
#define HAND_DESCRIPTION

#include <opencv2/opencv.hpp>
#include "handUtils.h"

class HandDescription 
{

	public:
	
		//-- Constructor
		HandDescription();
		

		void boundingRectangle(cv::Mat display);
		void contourExtraction(cv::Mat, cv::Mat);
		void angleControl();
   
   		void setHandAngle(double &); 
   		double getHandAngle ();
   		
   		std::vector< std::vector<cv::Point> > getContours();
   
   private: 
   
   		//-- Hand's parameters: hand angle & hand contour
   			
   		double _hand_angle; 
   		std::vector< std::vector<cv::Point> > _hand_contour;
   		cv::KalmanFilter kalmanFilterAngle;

   		
   		
};


#endif // HAND_DESCRIPTION
