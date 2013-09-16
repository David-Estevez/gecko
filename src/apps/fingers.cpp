#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std; 

void contours (cv:: Mat &, cv:: Mat &);
void background_substractor (cv:: Mat &, cv:: Mat &);


int main () 
{
	cv::Mat image, no_back_image, contour_image; 
	cv::VideoCapture cap(0); 

   if (!cap.isOpened())
        cerr << "Cannot open video device" << endl;


	while (1) 
	{
		cap>>image;
		cvtColor( image, image, CV_BGR2GRAY );
		cv::flip(image,image,1);

		
		background_substractor(image, no_back_image); 
		//contours (image); 


		
		//SHOW THE DIFFERENT IMAGES
		cv::imshow ("ORIGINAL IMAGE" , image); 
		//cv::imshow ("CONVEX HULL" , convex_image); 


		if (cv::waitKey(1)== 27)		//press ESC to exit camera
			break;          
	} 	                             
   
	return 0;
}

void background_substractor (cv:: Mat &src, cv:: Mat & result)
{



}

void contours (cv::Mat &src, cv :: Mat & result)
{

	int thresh = 100;
	int max_thresh = 255;



	cv::Mat canny_output;
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;


 	//blur( src_gray, src_gray, cv::Size(3,3) );

	/// Detect edges using canny

	cv::Canny( src, canny_output, thresh, thresh*2 );



	/// Find contours

	cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	result= cv::Mat::zeros( canny_output.size(), CV_8UC3 );

	/// Draw the contours

	for( int i = 0; i< contours.size(); i++ )
		   cv::drawContours( result, contours, i, cv:: Scalar (255,255,255), 2, 8, hierarchy, 0, cv::Point() );

}
