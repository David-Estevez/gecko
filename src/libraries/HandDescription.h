#ifndef HAND_DESCRIPTION
#define HAND_DESCRIPTION

#include <opencv2/opencv.hpp>
#include "handUtils.h"
#include "mouse.h"

class HandDescription 
{

public:

    //-- Constructor
    //-----------------------------------------------------------------------
    //! \brief Default constructor
    HandDescription();


    //-- Refresh the detected hand characteristics
    //-----------------------------------------------------------------------
    void operator ()( const cv::Mat src );
    void update( const cv::Mat src );


    //-- Get the characteristics of the hand:
    //-----------------------------------------------------------------------
    //! \brief Returns the angle of the box enclosing the hand
    double getHandAngle ();

    //! \brief Returns the position of the center of the hand
    std::pair <int, int> getCenterHand (cv:: Mat );
    std::pair <int, int> getCenterHand ( );

    //! \brief Returns the contours of the detected hand
    std::vector< std::vector<cv::Point> > getContours();

    //! \brief Returns the bounding box enclosing the detected hand
    cv::Rect getBoundingBox();

    //! \brief Returns the detected gesture:
    int getGesture();

    //! \brief Returns the number of fingers found:
    int getNumFingers();


    //-- Plot characteristics on some image:
    //--------------------------------------------------------------------------
    //! \brief Plots the rectangle around the hand on display
    void plotBoundingRectangle(const cv::Mat& src, cv::Mat& dst, bool rotated = false);

    //! \brief Plot the contours on display
    void plotContours(const cv::Mat& src, cv::Mat& dst );

    //! \brief Prints the angle gauge on a separate window
    void angleControl();


    //-- List of available gestures
    //---------------------------------------------------------------------------
    //! \note Edit/add more as you wish.
    static const int HAND_GESTURE_NONE = 0;
    static const int HAND_GESTURE_OPEN_PALM = 1;


private:
    //-- Functions that extract characteristics:
    //--------------------------------------------------------------------------
    void contourExtraction( const cv::Mat& src );
    void boundingBoxExtraction( const cv::Mat& src);

    void angleExtraction();
    void centerExtraction();
    void gestureExtraction();
    void numFingersExtraction();

    //-- Parameters that describe the hand:
    //--------------------------------------------------------------------------
    //! \brief Contains the angle of the box enclosing the hand
    double _hand_angle;

    //! \brief Center of the hand
    std::pair<int, int> _hand_center;

    //! \brief Contours of the candidates to be a hand
    std::vector< std::vector<cv::Point> > _hand_contour;

    //! \brief Box enclosing the hand
    cv::RotatedRect _hand_rotated_bounding_box;
    cv::Rect _hand_bounding_box;

    //! \brief Last detected gesture
    int _hand_gesture;

    //! \brief Number of fingers (visible)
    int _hand_num_fingers;


    //-- ROI of the hand, for gesture analysis:
    //-------------------------------------------------------------------------
    cv::Mat _hand_ROI;

    //-- Kalman filters for smoothing:
    //---------------------------------------------------------------------
    //! \brief Kalman filter for the angle of the box enclosing the hand
    cv::KalmanFilter kalmanFilterAngle;

    //! \brief Kalman filter for the center of the hand
    cv::KalmanFilter kalmanFilterCenter;



};


#endif // HAND_DESCRIPTION
