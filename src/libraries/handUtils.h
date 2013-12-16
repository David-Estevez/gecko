//------------------------------------------------------------------------------
//-- handUtils
//------------------------------------------------------------------------------
//--
//-- A collection of useful functions for hand gesture recognition
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

/*! \file handUtils.h
 *  \brief A collection of useful functions for hand gesture recognition
 *
 * \author David Estevez Fernandez ( http://github.com/David-Estevez )
 * \author Irene Sanz Nieto ( https://github.com/irenesanznieto )
 * \date Dec 12th, 2013
 */

#ifndef HANDUTILS_H
#define HANDUTILS_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "backgroundSubstractor.h"

/*! \fn drawCalibrationMarks
 * \brief Draws a calibration mark centered on the input image, and puts it on the output image
 *
 *  Currently the calibration mark is just a square
 *
 * \param input Image on which the mark is drawn
 * \param output Image where the result is stored
 * \param halfSide Half of the size of the mark side
 * \param color Color of the calibration square
 */
void drawCalibrationMarks(cv::Mat& input, cv::Mat& output, int halfSide = 20,  cv::Scalar color  = cv::Scalar( 0, 255, 0 ) );


/*!
 * \brief Creates three windows with the histograms for each channel of a 3-channel image
 * \param Source image for creating the histogram
 */
void drawHistogram(const cv::Mat& image);

/*!
 * \brief Creates three windows with the histograms for each HSV channel
 * \param Source image for creating the histogram
 */
void drawHistogramHSV( const cv::Mat& image);

/*!
 * \brief Filters the contours in a contour vector
 *
 * The criteria for filtering is the contour size, that has to be  larger than the lower threshold (for filtering small blobs) and
 * smaller than the upper threshold ( for filtering large background contours )
 *
 * \param srcContours Vector containing the original contours
 * \param handContour Vector containing the potential hand contours
 * \param min Lower threshold
 * \param max Upper threshold
 */
void filterContours(std::vector< std::vector<cv::Point> >& srcContours, std::vector< std::vector<cv::Point> >& handContour , const int min = 200, const int max = 1800);

/*!
 * \brief Finds the angle of a rotated rectangle with respect to the X axis
 *
 *  To find the angle, it looks for the larger side of the rectangle, and finds the angle of that side
 *  with respect to the X axis.
 *
 * \param boundingRect Rotated rectagle to find the angle
 * \return Angle of a rotated rectangle
 */
double getAngle( cv::RotatedRect boundingRect);

/*!
 * \brief backgroundSubs
 * \param bg
 */
void backgroundSubs(cv::Mat &, backgroundSubstractor & bg);


//! \brief Stores all the characteristics of a convexity defect in a more convenient way
struct ConvexityDefect
{

    cv::Point start;        //!< \brief Starting point of the defect
    cv::Point end;          //!< \brief Ending point of the defect
    cv::Point depth_point;  //!< \brief Deepest point of the defect
    double depth;           //!< \brief Depth of the defect

    int start_index;        //!< \brief Index of the starting point of the defect inside the original contour
    int end_index;          //!< \brief Index of the ending point of the defect inside the original contour
    int depth_point_index;  //!< \brief Index of the deepest point of the defect inside the original contour
};
typedef struct ConvexityDefect ConvexityDefect;


/*!
 * \brief Find the angle between two lines (using three points)
 *
 * The angle obtained here is the angle going from start-vertex segment to end-vertex segment
 *
 * \param start Starting point
 * \param end Ending point
 * \param vertex Vertex of the angle
 * \return  Angle between two lines
 */
float findAngle( cv::Point start, cv::Point end, cv::Point vertex);


/*!
 * \brief Prints a progress bar on the bottom of the screen
 * \param src Image on which to print the progress bar
 * \param dst Image to store the image with the progress bar
 * \param percentage Percentage of progress on the progress bar
 * \param color Color of the progress bar
 * \param thickness Thickness of the progress bar
 */
void printProgressBar( cv::Mat& src, cv::Mat& dst, float percentage, cv::Scalar color, int thickness = 15 );

#endif // HANDUTILS_H
