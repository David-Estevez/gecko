#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std; 


int main () 
{
	cv::Mat image; 
	cv::VideoCapture cap(0); 

   if (!cap.isOpened())
        cerr << "Cannot open video device" << endl;


	while (1) 
	{
		cap>>image;
		cv :: imshow ("VIDEO" , image); 

		if (cv::waitKey(1)== 27)		//press ESC to exit camera
			break;          
	} 	                             
   
	return 0;
}
