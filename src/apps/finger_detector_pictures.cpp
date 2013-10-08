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
	
	//-- Obtain the number of fingers
	int n_fingers= fingers.getFingers(); 
	
	//-- Print the number of fingers
	std::cout<<"Number of fingers: "<<n_fingers<<std::endl; 

	//-- Show the image
    cv::imshow("hand", image);
    
    //-- Wait for some key to be pressed	    
	cv::waitKey(0);
	return 0; 	    
}
