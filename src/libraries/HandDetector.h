#ifndef HAND_DETECTOR
#define HAND_DETECTOR

#include <opencv2/opencv.hpp>
#include "handUtils.h"

class HandDetector {
   public:
	//-- Constructors
	HandDetector();
	HandDetector( cv::Mat& ROI);

	//-- Calibration loop
	void calibrationLoop(cv::VideoCapture); 

	//-- Calibration functions
	void calibrate( cv::Mat& ROI);
	void calibrate( cv::Scalar lower_limit = cv::Scalar( 0, 58, 89), cv::Scalar upper_limit = cv::Scalar( 25, 173, 229) );
	void getCalibration( cv::Scalar& lower_limit, cv::Scalar& upper_limit);

	//-- Hand-detection
	void operator()(const cv::Mat& src, cv::Mat& dst);
	void filter_hand(const cv::Mat& src, cv::Mat& dst);
	
	//-- Get lower and upper level (calibration)
	cv::Scalar getLower(); 
	cv::Scalar getUpper(); 


    private:
	//-- Statistical functions:
	int average( cv::Mat& ROI);
	int median( cv::Mat& ROI);
	int stdDeviation( cv::Mat& ROI);

	//-- Filter contours:
	void filterContours( std::vector< std::vector < cv::Point > >& contours , std::vector< std::vector < cv::Point > >& filteredContours);

	//-- Filter face:
	//----------------------------------------------------------------------------------
	//! -- \brief Cascade classifier to detect faces:
	cv::CascadeClassifier faceDetector;

	//! -- \brief Initializes the face detector
	void initCascadeClassifier();

	//! -- \brief Removes the face from the src image
	void filterFace(const cv::Mat& src, cv::Mat& dstMask );


	//-- Skin hue calibration
	//-----------------------------------------------------------------------------------
	//-- HSV limits
	cv::Scalar lower_limit;
	cv::Scalar upper_limit;
	bool hue_invert;

	//-- HSV sigma multiplier
	int hue_sigma_mult;
	int sat_sigma_mult;
	int val_sigma_mult;

	cv::Scalar lower, upper;

	static const int halfSide=40;
};

#endif // HAND_DETECTOR
