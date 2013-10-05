#include <iostream>
#include "../libraries/FingerDetector.h"


int main (int argc, char * argv[])
{

	//-- Usage
	if (argc<2)
	{
		std::cerr<<"Usage: ./<executable> <picture>"<<std::endl;
		exit (-1);
	}
	
	cv::Mat image, display; 
	int num_fingers=0; 
	
	//-- Read the image and pass it to the fingerdetector constructor
	image=cv::imread(argv[1]);
	FingerDetector fingers(image, num_fingers); 
	
	
    cv::imshow("hand", image);
	    
	cv::waitKey(0);
	return 0; 	    
}
