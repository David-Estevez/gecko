 #include <opencv2/opencv.hpp>

class HandDetector {
   public:
	//-- Constructors
	HandDetector();
	HandDetector( cv::Mat& ROI);

	//-- Calibration functions
	void calibrate( cv::Mat& ROI);
	void calibrate( cv::Scalar lower_limit = cv::Scalar( 0, 58, 89), cv::Scalar upper_limit = cv::Scalar( 25, 173, 229) );
	void getCalibration( cv::Scalar& lower_limit, cv::Scalar& upper_limit);

	//-- Hand-detection
	void operator()(const cv::Mat& src, cv::Mat& dst);
	void find(const cv::Mat& src, cv::Mat& dst);

	//-- Find hand functions
	//! \todo Filter S only by lowest value (grey); filter V at both ends.
	void getContours(const cv::Mat& src, std::vector< std::vector < cv::Point > >& contours, cv::Mat& dst);


    private:
	//-- Statistical functions:
	int average( cv::Mat& ROI);
	int median( cv::Mat& ROI);
	int stdDeviation( cv::Mat& ROI);

	//-- Filter contours:
	void filterContours( std::vector< std::vector < cv::Point > >& contours , std::vector< std::vector < cv::Point > >& filteredContours);

	//-- HSV limits
	cv::Scalar lower_limit;
	cv::Scalar upper_limit;
	bool hue_invert;

	//-- HSV sigma multiplier
	int hue_sigma_mult;
	int sat_sigma_mult;
	int val_sigma_mult;

};
