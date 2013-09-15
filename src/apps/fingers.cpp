#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std; 

void contours (cv:: Mat &);

int main () 
{
	cv::Mat image; 
	cv::VideoCapture cap(0); 

   if (!cap.isOpened())
        cerr << "Cannot open video device" << endl;


	while (1) 
	{
		cap>>image;
		//cv::imshow ("VIDEO" , image); 

		contours (image); 

		if (cv::waitKey(1)== 27)		//press ESC to exit camera
			break;          
	} 	                             
   
	return 0;
}

void contours (cv::Mat &src)
{

	cv::Mat src_gray;
	int thresh = 100;
	int max_thresh = 255;



	cv::Mat canny_output;
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	cvtColor( src, src_gray, CV_BGR2GRAY );
 	blur( src_gray, src_gray, cv::Size(3,3) );


	cv::Canny( src_gray, canny_output, thresh, thresh*2 );
	/// Detect edges using canny


	/// Find contours
	cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	/// Draw contours
	cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );

	for( int i = 0; i< contours.size(); i++ )
		   cv::drawContours( drawing, contours, i, cv:: Scalar (255,255,255), 2, 8, hierarchy, 0, cv::Point() );
	 



	/// Draw the rectangle that contains the hand

	cv:: rectangle (drawing, cv::Point (0,0), cv:: Point (100, 100), cv:: Scalar (0,0,255), 5); 
 //void rectangle(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)


	/// Choose the contours within the rectangle



	/// Show the results

	cv::imshow( "Contours", drawing );

}
