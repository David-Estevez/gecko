//------------------------------------------------------------------------------
//-- HandDescriptor
//------------------------------------------------------------------------------
//--
//-- Finds the main characteristics of the hand from a binary image containing
//-- a hand silouette.
//--
//------------------------------------------------------------------------------
//--
//-- This file belongs to the "Gecko - Gesture Recognition" project
//-- (https://github.com/David-Estevez/gecko)
//--
//------------------------------------------------------------------------------
//-- Authors: David Estevez Fernandez
//--          Irene Sanz Nieto
//--
//-- Released under the GPL license (more info on LICENSE.txt file)
//------------------------------------------------------------------------------

/*! \file HandDescriptor.h
 *  \brief Finds the main characteristics of the hand from a binary image containing a hand silouette.
 *
 * \author David Estevez Fernandez ( http://github.com/David-Estevez )
 * \author Irene Sanz Nieto ( https://github.com/irenesanznieto )
 * \date Dec 12th, 2013
 */


#ifndef HAND_DESCRIPTOR
#define HAND_DESCRIPTOR

#include <opencv2/opencv.hpp>
#include "handUtils.h"
#include "mouse.h"

//-- Constants for the gesture recognition
//-----------------------------------------------------------------------
static const unsigned int GECKO_GESTURE_NONE = 0;
static const unsigned int GECKO_GESTURE_OPEN_PALM = 1;
static const unsigned int GECKO_GESTURE_CLOSED_PALM = 2;
static const unsigned int GECKO_GESTURE_VICTORY = 3;
static const unsigned int GECKO_GESTURE_GUN = 4;

class HandDescriptor
{

public:
    //-- Constructor
    //-----------------------------------------------------------------------
    //! \brief Default constructor
    HandDescriptor();


    //-- Refresh the detected hand characteristics
    //-----------------------------------------------------------------------
    void operator ()(const cv::Mat& src, const cv::Mat& skinMask );
    void update(const cv::Mat& src, const cv::Mat& skinMask );


    //-- Get the characteristics of the hand:
    //-----------------------------------------------------------------------
    //! \brief Returns true if a hand was found
    bool handFound();

    //! \brief Returns the angle of the box enclosing the hand
    double getHandAngle();
    double getHandAnglePredicted();
    double getHandAngleEstimated();

    //! \brief Returns the position of the center of the hand
    cv::Point getCenterHand( );
    cv::Point getCenterHandPredicted();
    cv::Point getCenterHandEstimated();

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
    void plotBoundingRectangle(const cv::Mat& src, cv::Mat& dst, bool rotated = true);

    //! \brief Plot the contours on display
    void plotContours(const cv::Mat& src, cv::Mat& dst );

    //! \brief Plot the center of the hand on display
    void plotCenter( const cv::Mat& src, cv::Mat& dst, bool show_corrected = true,  bool show_actual = true, bool show_predicted = true );

    //! \brief Prints the angle gauge on a separate window
    void angleControl( bool show_corrected = true,  bool show_actual = true, bool show_predicted = true );

    //! \brief Prints the maximum inscribed circle:
    void plotMaxInscribedCircle( cv::Mat& src, cv::Mat& dst, bool show_center = true, cv::Scalar color = cv::Scalar(0, 255, 0), int thickness = 1 );

    //! \brief Prints the minimum enclosing circle:
    void plotMinEnclosingCircle(cv::Mat& src, cv::Mat& dst, bool show_center = true, cv::Scalar color = cv::Scalar(255,0,255), int thickness = 1);

    //! \brief Plot the hand complex hull:
    void plotComplexHull(cv::Mat& src, cv::Mat& dst, bool show_points = false, cv::Scalar color = cv::Scalar( 0, 255, 255), int thickness = 1);

    //! \brief Plot convexity defects:
    void plotConvexityDefects(cv::Mat& src, cv::Mat& dst, bool draw_points = true);

    //! \brief Plot fingertip markers:
    void plotFingertips( cv::Mat& src, cv::Mat& dst, bool draw_lines = true, cv::Scalar color = cv::Scalar( 255, 0, 0) , int thickness = 2);

    //! \brief Plot hand interface:
    void plotHandInterface( cv::Mat& src, cv::Mat& dst );

private:
    //-- Functions that extract characteristics:
    //--------------------------------------------------------------------------
    void contourExtraction(const cv::Mat& skinMask);
    void boundingBoxExtraction( const cv::Mat& src);
    void handPalmExtraction();
    void ROIExtraction( const cv::Mat& src);
    void defectsExtraction();
    void fingerExtraction(const cv::Mat &src);
    void angleExtraction();
    void centerExtraction();
    void gestureExtraction();

    //-- Parameters that describe the hand:
    //--------------------------------------------------------------------------
    //! \brief Whether a hand was found or not:
    bool _hand_found;


    //! \brief Contains the actual angle of the box enclosing the hand
    double _hand_angle;

    //! \brief Contains the predicted angle by the kalman filter
    double _hand_angle_prediction;

    //! \brief Contains the corrected estimation by the kalman filter
    double _hand_angle_estimation;


    //! \brief Actual center of the hand
    cv::Point _hand_center;

    //! \brief Predicted center of the hand by the kalman filter
    cv::Point _hand_center_prediction;

    //! \brief Corrected estimation by the kalman filter
    cv::Point _hand_center_estimation;


    //! \brief Contours of the candidates to be a hand
    std::vector< std::vector<cv::Point> > _hand_contour;
    std::vector< std::vector<cv::Point> > _hand_contour_raw;


    //! \brief Box enclosing the hand
    cv::RotatedRect _hand_rotated_bounding_box;
    cv::Rect _hand_bounding_box;


    //! \brief Last detected gesture
    int _hand_gesture;


    //-- Describe fingers:
    //! \brief Number of fingers (visible)
    int _hand_num_fingers;

    //! \brief Position of the fingertips
    std::vector< cv::Point > _hand_fingertips;

    //! \brief Position of the finger line origin points
    std::vector< cv::Point > _hand_finger_line_origin;

    //-- Describe hand palm:
    //-------------------------------------------------------------------------
    //! \brief Radius of the max. inscribed circle
    double _max_circle_inscribed_radius;

    //! \brief Center of the max. incribed circle
    cv::Point _max_circle_incribed_center;

    //! \brief Radius of the min. enclosing circle
    float _min_enclosing_circle_radius;

    //! \brief Center of the min. enclosing circle
    cv::Point2f _min_enclosing_circle_center;

    //! \brief Complex hull of the hand
    std::vector< cv::Point > _hand_hull;

    //! \brief Convexity defects of the hand
    //std::vector< cv::Vec4i > _hand_convexity_defects;
    std::vector< ConvexityDefect > _hand_convexity_defects;



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


#endif // HAND_DESCRIPTOR
