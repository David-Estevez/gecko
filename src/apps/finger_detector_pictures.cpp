#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include "../libraries/HandDetector.h"
#include "../libraries/handUtils.h"
#include "../libraries/FingerDetector.h"


int main (int argc, char * argv[])
{
	if (argc<2)
	{
		std::cerr<<"Usage: ./<executable> <picture>"<<std::endl;
		exit (-1);
	}
	
	cv::Mat image, display; 
	
	image=cv::imread(argv[1]);
	
	
	if ( (int) handContour.size() > 0 )
	{
	    //-- Draw contours:
	    cv::drawContours( display, handContour, 0, cv::Scalar( 0, 0, 255), 1, 8);

	    //-- Draw rotated rectangle:
	    cv::RotatedRect minRect = cv::minAreaRect( handContour[0]);
	    cv::Point2f rect_points[4]; minRect.points( rect_points );
	    for( int j = 0; j < 4; j++ )
		cv::line( display, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255, 0, 0) , 1, 8 );



	    //-- Show hand ROI
	    cv::Rect rect  =  cv::boundingRect(handContour[0]);
	    cv::Mat ROI_hand = image(rect).clone();
	    std::cout<<"showing hand"<<std::endl;
	    cv::imshow("hand", ROI_hand);
	    
	    cv::waitKey(0);
	    
	}
	



}
