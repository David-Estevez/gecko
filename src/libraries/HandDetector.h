#include <opencv2/opencv.hpp>

class HandDetector {
   public:
	//-- Constructors
	HandDetector();
	HandDetector( cv::Mat& ROI);

	//-- Calibration functions
	//! \todo Check if values for hue are out of the limits
	void calibrate( cv::Mat& ROI);
	void calibrate( cv::Scalar lower_limit, cv::Scalar upper_limit);

	//-- Hand-detection
	void operator()(const cv::Mat& src, cv::Mat& dst);
	//! \todo If dst is not allocated, make a clone of src and use it
	void find(const cv::Mat& src, cv::Mat& dst);

	//-- Display configuration
	void setDisplayBoundingBox( const bool displayBoundingBox);
	void setDisplayBoundingRotRect( const bool displayBoundingRotRect);
	void setDisplayContour( const bool displayContour);
	void setDisplayThreshold( const bool displayThreshold);
	void setDisplay( const bool displayBoundingBox, const bool displayBoundingRotRect, const bool displayContour, const bool displayThreshold);

    private:
	//-- Statistical functions:
	int average( cv::Mat& ROI);
	int stdDeviation( cv::Mat& ROI);

	//-- Drawing result ?
	void draw( const cv::Mat& src, const cv::Mat& dst);

	//-- HSV limits
	cv::Scalar lower_limit;
	cv::Scalar upper_limit;

	//-- HSV sigma multiplier
	int hue_sigma_mult;
	int sat_sigma_mult;
	int val_sigma_mult;

	//-- Config flags
	bool displayBoundingBox;
	bool displayBoundingRotRect;
	bool displayContour;
	bool displayThreshold;
};
