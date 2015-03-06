//------------------------------------------------------------------------------
//-- Gecko_image_analyzer
//------------------------------------------------------------------------------
//--
//-- Demonstration of the Gecko main program features by analyzing static images
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

/*! \file image_analyzer.cpp
 *  \brief Demonstration of the Gecko main program features by analyzing static images
 *
 * \author David Estevez Fernandez ( http://github.com/David-Estevez )
 * \author Irene Sanz Nieto ( https://github.com/irenesanznieto )
 * \date Dec 12th, 2013
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "HandDetector.h"
#include "HandDescriptor.h"

int main( int argc, char * argv[] )
{
    if ( argc < 2)
    {
        std::cout << "Gecko - Gesture Recognition\n\nUsage: gecko_image_analyzer <image> <image to save>(optional)\n" << std::endl;
        return -1;
    }

    //-- Load image from file
    std::cout << "Opening " << cv::String(argv[1]) << std::endl;
    cv::Mat image = cv::imread( cv::String(argv[1]), CV_LOAD_IMAGE_COLOR);
    cv::Mat processed;
    cv::Mat display;

    HandDetector handDetector;           //-- To find the hand
    HandDescriptor hand_descriptor;     //-- Object that will store the parameters of the hand



    //-- Process it
    //-----------------------------------------

    handDetector(image, processed);

    //-- Contour extraction
    hand_descriptor( processed );


    //-- Hand's angle
    std::cout << "Angle: [" << hand_descriptor.getHandAngle() << "]" << std::endl;


    //-- Plot things on the image
    //--------------------------------------------
    if ( display.total() == 0)
        display = image.clone();

    //-- Plot hand interface
    //--------------------------------------------
    hand_descriptor.plotHandInterface(display, display);


    //-- Show gesture marker
    //-------------------------------------------
    cv::Scalar color;
    int fill = CV_FILLED;
    switch( hand_descriptor.getGesture() )
    {
    case GECKO_GESTURE_OPEN_PALM: color = cv::Scalar( 255, 0, 0); break;
    case GECKO_GESTURE_VICTORY: color = cv::Scalar( 0, 255, 0); break;
    case GECKO_GESTURE_GUN: color = cv::Scalar( 0, 0, 255); break;
    case GECKO_GESTURE_CLOSED_FIST: color = cv::Scalar(255, 255, 255); break;
    default: color = cv::Scalar( 255, 255, 255); fill = 1;
    }

    if ( hand_descriptor.handFound() )
    {
        cv::circle( display, cv::Point( display.cols - 50, display.rows - 25 ), 7.5, color, fill );
    }


    //-- Show detected faces
    //--------------------------------------------
    handDetector.drawFaceMarks( display, display );

    //-- Show feedback image
    //----------------------------------------------------------------
    cv::imshow( "Original  image", image);
    cv::imshow( "Gecko", display);

    //-- Save file
    //----------------------------------------------------------------
    if (argc == 3)
    {
        cv::imwrite( std::string(argv[2]), display );
    }
    //-- Wait
    //----------------------------------------------------------------
    cv::waitKey(-1);

    return 0;
}
