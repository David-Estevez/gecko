#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std; 

class myBackgroundSubstractor: public cv::BackgroundSubtractorMOG2
	{
	public:
		void setbackgroundRatio(float a){backgroundRatio = a;}
};

void contours_extraction  (cv:: Mat &, cv:: Mat &);
void convex_hull (cv::Mat &, cv :: Mat &);

int main () 
{

	cv::Mat image, no_back_image, final_image; 
	cv::VideoCapture cap(0); 
	cap.set(CV_CAP_PROP_BRIGHTNESS,1);

	myBackgroundSubstractor bg; 
	bg.set("nmixtures",3);// set number of gaussian mixtures
	bg.set("detectShadows", false); //if false: turn shadow detection off
	bg.setbackgroundRatio(0.0001);
	vector<vector<cv::Point> > contours;

   if (!cap.isOpened())
        cerr << "Cannot open video device" << endl;


	while (1) 
	{
		cap>>image;
		cvtColor( image, image, CV_BGR2GRAY );
		cv::flip(image,image,1);

		bg.operator ()(image,no_back_image);

		cv::erode(no_back_image,no_back_image,cv::Mat());
		cv::dilate(no_back_image,no_back_image,cv::Mat());
		cv::findContours(no_back_image,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		cv::drawContours(image,contours,-1,cv::Scalar(0,0,255),2);
		image.copyTo(no_back_image, no_back_image);



//contours
		//contours_extraction (no_back_image, final_image); 


//convex_hull 
		convex_hull ( no_back_image, final_image); 

		//SHOW THE DIFFERENT IMAGES
		//cv::imshow ("ORIGINAL IMAGE" , image);
		//cv::imshow ("NO BACKGROUND IMAGE" , no_back_image); 
		cv::imshow ("FINAL IMAGE" , final_image);


		if (cv::waitKey(1)== 27)		//press ESC to exit camera
			break;          
	} 	                             
   
	return 0;
}



void convex_hull (cv::Mat &src_gray, cv :: Mat & result)
{/*
int thresh = 100;
 int max_thresh = 255;
 RNG rng(12345);

   cv::Mat threshold_output;
   vector<vector<cv::Point> > contours;
   vector<cv::Vec4i> hierarchy;

   /// Detect edges using Threshold
   cv::threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );

   /// Find contours
   cv::findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

   /// Find the convex hull object for each contour
   vector<vector<cv::Point> >hull( contours.size() );
   for( int i = 0; i < contours.size(); i++ )
      {  convexHull( Mat(contours[i]), hull[i], false ); }

   /// Draw contours + hull results
   cv::Mat drawing =cv:: Mat::zeros( threshold_output.size(), CV_8UC3 );
   for( int i = 0; i< contours.size(); i++ )
      {
        cv::Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       cv:: drawContours( drawing, contours, i, color, 1, 8, vector<cv::Vec4i>(), 0, Point() );
        cv::drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
      }

   /// Show in a window
   namedWindow( "Hull demo", CV_WINDOW_AUTOSIZE );
   imshow( "Hull demo", drawing );


*/
}



void contours_extraction  (cv::Mat &src, cv :: Mat & result)
{

	int thresh = 100;
	int max_thresh = 255;



	cv::Mat canny_output;
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;


 	blur( src, src, cv::Size(3,3) );

	/// Detect edges using canny

	cv::Canny( src, canny_output, thresh, thresh*2 );



	/// Find contours

	cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	result= cv::Mat::zeros( canny_output.size(), CV_8UC3 );

	/// Draw the contours

	for( int i = 0; i< contours.size(); i++ )
		   cv::drawContours( result, contours, i, cv:: Scalar (255,255,255), 2, 8, hierarchy, 0, cv::Point() );

}
