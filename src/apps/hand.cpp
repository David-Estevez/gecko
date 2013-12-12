#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../libraries/HandDetector.h"
#include "../libraries/HandDescription.h"
#include "../libraries/handUtils.h"
#include "../libraries/FingerDetector.h"
#include "../libraries/mouse.h"
#include "../libraries/StateMachine.h"
#include "../libraries/AppLauncher.h"
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
        return(1);
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

    //-- State machine for tracking the cursor
    StateMachine cursor_SM( GECKO_GESTURE_OPEN_PALM, 3, 5);

    //-- State machine for clicking
    StateMachine click_SM( GECKO_GESTURE_CLOSED_PALM, 15, 5);

    //-- AppLauncher for launching programs
    AppLauncher launcher( "../data/apps.config", 30, 5);

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
            break;
            //frame = cv::imread("../data/hand1.jpg");
            //frame = cv::imread( "../data/3dedos.jpg");

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
            case GECKO_GESTURE_CLOSED_PALM: color = cv::Scalar(255, 255, 255); break;
            default: color = cv::Scalar( 255, 255, 255); fill = 1;
        }

        if ( hand_descriptor.handFound() )
        {
            cv::circle( display, cv::Point( display.cols - 50, display.rows - 25 ), 7.5, color, fill );
        }


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
            ss << "Commands enabled";
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
        //-- Command mode actions
        //-----------------------------------------------------------------------------------------------------
        if ( debugValue == 2 && hand_descriptor.handFound() )
        {
            //-- Move Cursor
            //-----------------------------------------------------------------------------------------------------

            //-- Check the state machine
            cursor_SM.update( hand_descriptor.getGesture() );

            if ( cursor_SM.getFound() )
            {
                //-- Show hand center of screen
                hand_descriptor.plotCenter( display, display );

                //-- Calculate relative position and move there:
                const int border = 50; //-- Leave a 50 px border around image
//                cv::Point win_up_left = cv::Point( border, border);
//                cv::Point win_down_right = cv::Point( display.cols - border, display.rows - border);
//                cv::rectangle( display, win_up_left, win_down_right, cv::Scalar(255,255,255), 2);

                cv::Point hand_center = hand_descriptor.getCenterHandEstimated();

                std::pair< float, float> relativeCoordinates;

                //-- Find if hand center is out of the window ( and limit it to the window border )
                if ( hand_center.x < border ) hand_center.x = border;
                if ( hand_center.y < border ) hand_center.y = border;
                if ( hand_center.x > display.cols - border ) hand_center.x = display.cols - border;
                if ( hand_center.y > display.rows - border ) hand_center.y = display.rows - border;

                //relativeCoordinates.first = hand_center.x /  (double) frame.cols;
                //relativeCoordinates.second= hand_center.y / (double) frame.rows;

                relativeCoordinates.first = (hand_center.x - border) /  (double)( frame.cols - 2 * border);
                relativeCoordinates.second = (hand_center.y - border) /  (double)( frame.rows - 2 * border);

                moveMousePercentage( relativeCoordinates );
            }
            else
            {
                printProgressBar( display, display, cursor_SM.getPercentageMatches(), cv::Scalar( 255, 0, 0) );
            }

            //-- Click action:
            //---------------------------------------------------------------------------------------------------
            //-- Check the state machine
            click_SM.update( hand_descriptor.getGesture() );

            if ( click_SM.getFound() )
            {
                click();
                click_SM;
            }
            else
                printProgressBar( display, display, click_SM.getPercentageMatches(), cv::Scalar(255, 255, 255) );



            //-- Run apps
            //----------------------------------------------------------------------------------------------------

            //-- Update the launcher state machines
            launcher.update( hand_descriptor.getGesture() );

            for (int i = 0; i < launcher.getNumberOfCommands(); i++)
                if ( !launcher.getFound(i) && launcher.getPercentageMatches(i) != 0)
                {
                    cv::Scalar bar_color;
                    switch( launcher.getValueToTrackAt(i) )
                    {
                    case GECKO_GESTURE_VICTORY: bar_color = cv::Scalar( 0, 255, 0); break;
                    case GECKO_GESTURE_GUN: bar_color = cv::Scalar( 0, 0, 255); break;
                    case GECKO_GESTURE_CLOSED_PALM: bar_color = cv::Scalar(255, 255, 255); break;
                    }

                    printProgressBar( display, display, launcher.getPercentageMatches(i), bar_color );
                }

        }


        //-----------------------------------------------------------------------------------------------------
        //-- Show angle gauge
        //----------------------------------------------------------------------------------------------------
        hand_descriptor.angleControl();


        //-----------------------------------------------------------------------------------------------------
        //-- Show feedback image
        //-----------------------------------------------------------------------------------------------------
        cv::imshow( "Gecko", display);

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
        case 'c':
            click();
        default:
            continue;
        }
    }

    return 0;
}
