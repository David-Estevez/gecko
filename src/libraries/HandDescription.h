#ifndef HAND_DESCRIPTION
#define HAND_DESCRIPTION

#include <opencv2/opencv.hpp>
#include "handUtils.h"
#include "mouse.h"

class HandDescription 
{

public:

    //! \brief Default constructor
    HandDescription();

    //! \brief Plots the rectangle around the hand on display
    void boundingRectangle(cv::Mat display);

    //! \brief Extract the countours of a image
    void contourExtraction(cv::Mat, cv::Mat);

    //! \brief Prints the angle gauge on a separate window
    void angleControl();

    //! \brief Returns the angle of the box enclosing the hand
    double getHandAngle ();

    //! \brief Returns the position of the center of the hand
    std::pair <int, int> getCenterHand (cv:: Mat );

    //! \brief Returns the contours of the detected hand
    std::vector< std::vector<cv::Point> > getContours();


private:
    //! \brief Contains the angle of the box enclosing the hand
    double _hand_angle;

    //! \brief Contours of the candidates to be a hand
    std::vector< std::vector<cv::Point> > _hand_contour;

    //! \brief Kalman filter for the angle of the box enclosing the hand
    cv::KalmanFilter kalmanFilterAngle;

    //! \brief Kalman filter for the center of the hand
    cv::KalmanFilter kalmanFilterCenter;



};


#endif // HAND_DESCRIPTION
