#include <opencv2/opencv.hpp>

class HandDetector {
    private:
	cv::VideoCapture capture;
	void (*function)(cv::Mat&, cv::Mat&);

    public:
	HandDetector(const std::string& name);
	HandDetector(const int& device);

	void setFunction( void (*function)(cv::Mat&, cv::Mat&) );

	void nextFrame();
};
