#ifndef HAND_DETECTOR
#define HAND_DETECTOR

#include <opencv2/opencv.hpp>
#include "handUtils.h"

class HandDetector {
   public:
	//-- Constructors
	HandDetector();
	HandDetector( cv::Mat& ROI);

    //-- Destructors
    ~HandDetector();

	//-- Calibration loop
    void calibrationLoop(cv::VideoCapture);

    void defaultValues(cv::VideoCapture cap);

    void customValues(cv::VideoCapture cap);


	//-- Calibration functions
	void calibrate( cv::Mat& ROI);
	void calibrate( cv::Scalar lower_limit = cv::Scalar( 0, 58, 89), cv::Scalar upper_limit = cv::Scalar( 25, 173, 229) );
	void getCalibration( cv::Scalar& lower_limit, cv::Scalar& upper_limit);

	//-- Hand-detection
    void operator()(cv::Mat& src, cv::Mat& dst);
    void filter_hand(cv::Mat& src, cv::Mat& dst);

	//-- Face-tracking
	std::vector< cv::Rect >& getLastFacesPos();
	void drawFaceMarks( const cv::Mat& src, cv::Mat& dst , cv::Scalar color = cv::Scalar(0, 255, 0), int thickness = 1  );
	
	//-- Get lower and upper level (calibration)
	cv::Scalar getLower(); 
	cv::Scalar getUpper(); 


    private:
	//-- Statistical functions:
	//! \todo OpenCV already has this functions, so use then instead.
	int average( cv::Mat& ROI);
	int median( cv::Mat& ROI);
	int stdDeviation( cv::Mat& ROI);

	//-- Hand filtering functions:
    void backgroundSubstraction(cv::Mat& src, cv::Mat& dst);
	void threshold( const cv::Mat& src, cv::Mat& dst);
	void filterBlobs( const cv::Mat& src, cv::Mat& dst);

	//-- Filter contours:
	void filterContours( std::vector< std::vector < cv::Point > >& contours , std::vector< std::vector < cv::Point > >& filteredContours);

    //-- Background substractor:
    //---------------------------------------------------------------------------------
    backgroundSubstractor bg ;
    void initBackgroundSubstractor();

	//-- Filter face:
	//----------------------------------------------------------------------------------
	//! -- \brief Cascade classifier to detect faces:
	cv::CascadeClassifier faceDetector;

	//! -- \brief Position of the last faces found:
	std::vector< cv::Rect > lastFacesPos;

	//! -- \brief Resize the rectangles.
	double factorX, factorY;

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



	//-- Calibration box size
    static const int halfSide=40;
};

#endif // HAND_DETECTOR
