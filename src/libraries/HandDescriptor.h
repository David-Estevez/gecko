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
 */


#ifndef HAND_DESCRIPTOR
#define HAND_DESCRIPTOR

#include <opencv2/opencv.hpp>
#include "handUtils.h"
#include "mouse.h"



/*! \class HandDescriptor
 *  \brief Finds the main characteristics of the hand from a binary image containing a hand silouette.
 *
 *  To look for or to update the stored characteristics of the hand and its gesture, the update member
 *  or the operator () can be called. One can then know if a hand was found using handFound member, prior
 *  to retrieving any of the hand characteristics / gesture.
 */
class HandDescriptor
{

public:
    //-- Constants for the gesture recognition
    //-----------------------------------------------------------------------
    //! \brief No gesture found
    static const unsigned int GECKO_GESTURE_NONE;
    //! \brief Open palm, 4 or 5 fingers seen
    static const unsigned int GECKO_GESTURE_OPEN_PALM;
    //! \brief Fist closed
    static const unsigned int GECKO_GESTURE_CLOSED_FIST;
    //! \brief Sign of victory, index and middle fingers in a < 60º angle
    static const unsigned int GECKO_GESTURE_VICTORY;
    //! \brief Index and thumb at right angles, making a gun
    static const unsigned int GECKO_GESTURE_GUN;

    //-- Constructor
    //-----------------------------------------------------------------------
    //! \brief Default constructor
    HandDescriptor();


    //-- Refresh the detected hand characteristics
    //-----------------------------------------------------------------------
    /*! \brief Update the internal characteristics stored
     *
     *  This is a wrapper of the update function, to call it in a more
     *  intuitive way.
     *
     *  \param skinMask Binary image containing the skin zones of hand candidates
     */
    void operator ()(const cv::Mat& skinMask );

    /*! \brief Update the internal characteristics stored
     *
     *  Extracts all the hand characteristics and guesses the current gesture
     *
     *  \param skinMask Binary image containing the skin zones of hand candidates
     */
    void update(const cv::Mat& skinMask );


    //-- Get the characteristics of the hand:
    //-----------------------------------------------------------------------
    //! \brief Returns true if a hand was found
    bool handFound();

    //! \brief Returns the angle of the box enclosing the hand
    double getHandAngle();
    //! \brief Returns the angle of the box enclosing the hand predicted by the Kalman filter
    double getHandAnglePredicted();
     //! \brief Returns the angle of the box enclosing the hand estimated by the Kalman filter after updating the prediction with the actual value
    double getHandAngleEstimated();

    //! \brief Returns the position of the center of the hand
    cv::Point getCenterHand();
    //! \brief Returns the position of the center of the hand predicted by the Kalman filter
    cv::Point getCenterHandPredicted();
    //! \brief Returns the position of the center of the hand estimated by the Kalman filter after updating the prediction with the actual value
    cv::Point getCenterHandEstimated();

    //! \brief Returns the contours of the detected hand
    std::vector< std::vector<cv::Point> > getContours();

    //! \brief Returns the bounding box enclosing the detected hand
    cv::Rect getBoundingBox();

    //! \brief Returns the detected gesture
    int getGesture();

    //! \brief Returns the number of fingers found
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
    /*! \brief Extract the hand contours from a binary image containing hand candidates
     *
     *  After the contour extraction it filters out the smaller contours, that are likely to
     *  be noise, and carries a polygon approximation to reduce the number of points in the
     *  contour.
     *
     *  \param skinMask Binary image containing the hand candidates, previously filtered by a
     *  HandDetector object.
     */
    void contourExtraction(const cv::Mat& skinMask);

    //! \brief Extracts the bounding boxes around the hand contour ( rectangle and rotated rectange)
    void boundingBoxExtraction();

    //! \brief Finds the maximum inscribed circle of the hand contour, which describes the hand palm
    void handPalmExtraction();

    /*! \brief Extracts a mask of the region of interest in which the hand is contained
     *  \param src Binary image containing the hand candidates, to extract the dimensions of the mask image
     */
    void ROIExtraction( const cv::Mat& src);

    //! \brief Finds the convexity defects of the hand convex hull, that are used to find the fingers
    void defectsExtraction();

    //! \brief Extracts the number, position and orientation of the fingers
    void fingerExtraction();

    //! \brief Extracts the hand angle from its bounding box and a Kalman Filter
    void angleExtraction();

    //! \brief Extracts the hand center and applies a Kalman Filter for improved stability
    void centerExtraction();

    //! \brief Guesses the hand gesture using the hand characteristic data previouly found
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


    //! \brief Minimum RotatedRect enclosing the hand
    cv::RotatedRect _hand_rotated_bounding_box;

    //! \brief Minimum Rect enclosing the hand
    cv::Rect _hand_bounding_box;


    //! \brief Last detected gesture, coded as an integer (see constants for correspondence between integer and gesture)
    int _hand_gesture;



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
    std::vector< ConvexityDefect > _hand_convexity_defects;


    //! \brief Mask containing ROI of the hand
    cv::Mat _hand_ROI;


    //-- Kalman filters for smoothing:
    //---------------------------------------------------------------------
    //! \brief Kalman filter for the angle of the box enclosing the hand
    cv::KalmanFilter kalmanFilterAngle;

    //! \brief Kalman filter for the center of the hand
    cv::KalmanFilter kalmanFilterCenter;


};


#endif // HAND_DESCRIPTOR
