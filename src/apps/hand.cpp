#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../libraries/HandDetector.h"
#include "../libraries/HandDescription.h"
#include "../libraries/handUtils.h"
#include "../libraries/FingerDetector.h"
#include "../libraries/mouse.h"
#include <unistd.h>

int main( int argc, char * argv[] )
{
    //-- Setup video
    //--------------------------------------------------------------
    cv::VideoCapture cap;

    //-- Open video source
    if ( argc > 1)
    {
        cap.open( argv[1] );
    }
    else
    {
        cap.open(-1);
        cap.set(CV_CAP_PROP_BRIGHTNESS, 0.5);
    }

    //-- Check if open
    if ( !cap.isOpened() )
    {
        std::cerr << "Device could not be opened." << std::endl;
        //return(1);
    }

    //-- Get frame rate
    //double rate = cap.get( CV_CAP_PROP_FPS);
    //int delay = 1000/rate;
    int delay = 24; //-- Force 24 ms delay


    //-- Declare variables
    //--------------------------------------------------------------------
    //-- Program control
    bool stop = false;
    int debugValue = 0;

    //-- To find the hand
    HandDetector handDetector;

    //-- Object that will store the parameters of the hand
    HandDescription hand_descriptor;

    //-- Initial screen
    cv::Mat init_screen=cv::imread("../img/init.png");

    while (1)
    {
        cv::imshow("GECKO", init_screen);
        char key =  cv::waitKey(delay);
        if ( key == 10 || key == 13 )
        {
            cv::destroyWindow("GECKO");
            break;
        }
        else if (key==27 || key=='q')
            exit(0);
    }


    //-- Menu

    cv::Mat menu=cv::imread("../img/menu.png");
    while (1)
    {
        cv::imshow("MENU", menu);
        //-- Wait for user confirmation
        char key =  cv::waitKey(delay);
        if ( key == '1' || key == '2' )
        {
            cv::destroyWindow("MENU");
            if (key=='1')
                handDetector.defaultValues(cap);
            else if (key=='2')
                handDetector.customValues(cap);

            break;
        }
        else if (key==27 || key=='q')
            exit(0);
    }




    //-- Main loop
    //--------------------------------------------------------------------
    stop = false;
    while(!stop)
    {
        //------------------------------------------------------------------------------------------------------
        //-- Get current frame
        //-------------------------------------------------------------------------------------------------------
        cv::Mat frame, display;
        if ( ! cap.read( frame ) )
            //break;
            //frame = cv::imread("../data/hand1.jpg");
            frame = cv::imread( "../data/3dedos.jpg");

        cv::flip(frame,frame,1);

        //------------------------------------------------------------------------------------------------------
        //-- Process it
        //------------------------------------------------------------------------------------------------------
        cv::Mat processed;
//        switch( debugValue )
//        {
//        case 0: case 2:
//            handDetector.calibrate();
//            handDetector( frame, processed);
//            break;

//        case 1:
//            handDetector.calibrate( handDetector.getLower(), handDetector.getUpper());
//            handDetector( frame, processed);
//            break;

//        default:
//            processed = frame;
//            break;
//        }

        handDetector(frame, processed);

        //-- Contour extraction
        hand_descriptor( frame, processed );


        //-- Hand's angle
        std::cout << "[" << hand_descriptor.getHandAngle() << "]" << std::endl;


        //--------------------------------------------------------------------------------------------------
        //-- Plot things on the image
        //--------------------------------------------------------------------------------------------------
        if ( display.total() == 0)
            display = frame.clone();


        //-- Plot contours on image
        //--------------------------------------------
        //hand_descriptor.plotBoundingRectangle( display, display );
        hand_descriptor.plotContours(display, display);
        hand_descriptor.plotMaxInscribedCircle(display, display );
        hand_descriptor.plotMinEnclosingCircle(display, display);
        hand_descriptor.plotComplexHull( display, display, true);
        hand_descriptor.plotConvexityDefects(display, display);
        hand_descriptor.plotFingertips(display, display);


        //-- Adding text:
        //--------------------------------------------
        std::stringstream ss;
        ss << "Mode: ";

        switch( debugValue)
        {
        case 0:
            ss << "Default values";
            break;

        case 1:
            ss << "Custom values->" << handDetector.getLower() << " " << handDetector.getUpper();
            break;

        case 2:
            ss << "Tracking hand";
            break;

        }
        ss << " Angle: " <<hand_descriptor.getHandAngle()<< "";
        std::string text = ss.str();
        cv::putText( display, text.c_str(), cv::Point(0, 18),
                     cv::FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(0, 0, 255));


        //-- Show detected faces
        //--------------------------------------------
        handDetector.drawFaceMarks( display, display );


        //-----------------------------------------------------------------------------------------------------
        //-- Move Cursor
        //-----------------------------------------------------------------------------------------------------
        if ( debugValue == 2)
        {
            //-- Show hand center of screen
            hand_descriptor.plotCenter( display, display );

            //-- Calculate relative position and move there:
            std::pair< int, int> hand_center = hand_descriptor.getCenterHandEstimated();

            std::pair< float, float> relativeCoordinates;
            relativeCoordinates.first = hand_center.first /  (double) frame.cols;
            relativeCoordinates.second= hand_center.second / (double) frame.rows;

            moveMousePercentage( relativeCoordinates );
        }


        //-----------------------------------------------------------------------------------------------------
        //-- Show angle gauge
        //----------------------------------------------------------------------------------------------------
        hand_descriptor.angleControl();


        //-----------------------------------------------------------------------------------------------------
        //-- Show feedback image
        //-----------------------------------------------------------------------------------------------------
        cv::imshow( "Processed Stream", display);

        //-----------------------------------------------------------------------------------------------------
        //-- Decide what to do next depending on key pressed
        //-----------------------------------------------------------------------------------------------------
        char key = (char) cv::waitKey(delay);
        switch( key)
        {
//        case 'f': //-- Filtered frame
//            debugValue = 0;
//            break;
//        case 'd': //-- hardcoded filter
//            debugValue = 1;
            break;
        case 'k': //-- Move mouse
            debugValue = 2;
            break;
        case (char) 27:
            stop = true;
            break;
        default:
            continue;
        }
    }

    return 0;
}
