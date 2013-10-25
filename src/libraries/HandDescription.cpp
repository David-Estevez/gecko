#include "HandDescription.h"

//-- Initialization of the private parameters in the constructor
HandDescription:: HandDescription()
{
	_hand_angle=0; 
	
	//-- Kalman filter setup for estimating hand angle:
    //-----------------------------------------------------------------------
    //-- Create filter:
    kalmanFilterAngle.init( 2, 1, 0);
    kalmanFilterAngle.transitionMatrix = *( cv::Mat_<float>(2, 2) << 1, 1,
								     0, 1);

    //-- Initial state:
    kalmanFilterAngle.statePre.at<float>(0) = 90; //-- initial angle
    kalmanFilterAngle.statePre.at<float>(1) = 0;  //-- initial angular velocity

    //-- Set the rest of the matrices:
    cv::setIdentity( kalmanFilterAngle.measurementMatrix );
    cv::setIdentity( kalmanFilterAngle.processNoiseCov, cv::Scalar::all(0.0001));
    cv::setIdentity( kalmanFilterAngle.measurementNoiseCov, cv::Scalar::all(0.1));
    cv::setIdentity( kalmanFilterAngle.errorCovPost, cv::Scalar::all(0.1));


}


//-- Set - get functions for private parameters
void HandDescription:: setHandAngle (double & value)
{
	_hand_angle=value; 
}

double HandDescription:: getHandAngle ()
{
	return _hand_angle; 
}


//-- Draws the bounding rectangle of the ROI
void HandDescription::boundingRectangle(cv::Mat display)
{
	    //-- Draw rotated rectangle:
	    cv::RotatedRect minRect = cv::minAreaRect( _hand_contour[0]);
	    cv::Point2f rect_points[4]; minRect.points( rect_points );
	    for( int j = 0; j < 4; j++ )
		cv::line( display, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255, 0, 0) , 1, 8 );

	    double newHandAngle = getAngle(minRect);
	    _hand_angle = newHandAngle < 0 ? _hand_angle : newHandAngle;
}


//-- Extracts the contour of the ROI and stores the hand's parameters in the private variables
void HandDescription :: contourExtraction(cv::Mat frame, cv::Mat processed)
{
	filteredContour( processed, _hand_contour);

	cv::Mat display; 
	
	if ( display.total() == 0)
	    display = frame.clone();

	if ( (int) _hand_contour.size() > 0 )
	{
	    //-- Draw contours:
	    cv::drawContours( display, _hand_contour, 0, cv::Scalar( 0, 0, 255), 1, 8);
	    //cv::fillConvexPoly( dst, contours[largestId], cv::Scalar( 255, 255, 255));
	    
	    HandDescription:: boundingRectangle(display);

	    //-- Show hand ROI
	    cv::Rect rect  =  cv::boundingRect(_hand_contour[0]);
	    cv::Mat ROI_hand = frame( rect).clone();
	    cv::imshow("hand", ROI_hand);
	    
//	    //-- Finger detector 
//	    
//	    int fingers=0; 
//	    FingerDetector (ROI_hand, fingers);
//	    
//	    cv::imshow("fingers", ROI_hand);

	}
	else
	    std::cerr << "No contours found!" << std::endl;
	    
}


void HandDescription:: angleControl()
{
	//-----------------------------------------------------------------------------------------------------
	//-- Print "Angle control"
	//-----------------------------------------------------------------------------------------------------
	
	//-- Matrix that shows the gauge
	cv::Mat gauge = cv::Mat::zeros( 100, 200, CV_8UC3);
	
	//-- Create matrix for storing the measurement (measured position of hand)
    cv::Mat_<float> angleMeasurement;
    angleMeasurement(1, 1);
    angleMeasurement.setTo( cv::Scalar(0));

	//-- Predict angle with Kalman filter:
	cv::Mat anglePrediction = kalmanFilterAngle.predict();

	//-- Measure:
	angleMeasurement(0) = _hand_angle;

	//-- Correct prediction:
	cv::Mat angleEstimation = kalmanFilterAngle.correct( angleMeasurement );

	//-- Define gauge:
	int gauge_l = 60;
	cv::Point gaugeOrigin( 200/2, 80 );

	//-- Calculate actual end:
	double rad_ang = _hand_angle * 3.1415 / 180.0;
	int x_coord = gauge_l * cos( rad_ang);
	int y_coord = gauge_l * sin( rad_ang);
	cv::Point gaugeEnd( 200/2 + x_coord , 80 - y_coord);

	//-- Calculate predicted end:
	double rad_ang_predicted = anglePrediction.at<float>(0) * 3.1415 / 180.0;
	int x_coord_predicted = gauge_l * cos( rad_ang_predicted);
	int y_coord_predicted = gauge_l * sin( rad_ang_predicted);
	cv::Point predictedGaugeEnd( 200/2 + x_coord_predicted , 80 - y_coord_predicted);

	//-- Calculate estimated end:
	double rad_ang_estimated = angleEstimation.at<float>(0) * 3.1415 / 180.0;
	int x_coord_estimated = gauge_l * cos( rad_ang_estimated);
	int y_coord_estimated = gauge_l * sin( rad_ang_estimated);
	cv::Point estimatedGaugeEnd( 200/2 + x_coord_estimated , 80 - y_coord_estimated);

	//-- Show the gauge(s):
	cv::line( gauge, gaugeOrigin, gaugeEnd, cv::Scalar( 255, 0, 0), 1); //-- Actual angle
	cv::line( gauge, gaugeOrigin, predictedGaugeEnd, cv::Scalar( 0, 255, 0), 2); //-- Predicted angle
	cv::line( gauge, gaugeOrigin, estimatedGaugeEnd, cv::Scalar( 0, 0, 255)), 3; //-- Estimated angle
	
	cv::imshow( "Gauge", gauge);}
