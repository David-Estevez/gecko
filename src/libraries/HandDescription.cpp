#include "HandDescription.h"

//-- Initialization of the private parameters in the constructor
HandDescription:: HandDescription()
{
    //-- Initalize hand parameters
    //-----------------------------------------------------------------------
    _hand_angle=0;
    _hand_center = std::pair <int, int> (0, 0);
    _hand_bounding_box = cv::Rect();
    _hand_gesture = HAND_GESTURE_NONE;
    _hand_num_fingers = -1;


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



    //-- Kalman filter for estimating hand center
    //---------------------------------------------------------------------

    //-- Create filter:
    kalmanFilterCenter.init( 4, 2, 0);
    kalmanFilterCenter.transitionMatrix = *( cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
								0, 1, 0, 1,
								0, 0, 1, 0,
								0, 0, 0, 1);
    //-- Get mouse position:
    //! \todo Change this for screen center?
    int initial_mouse_x, initial_mouse_y;
    getMousePos( initial_mouse_x, initial_mouse_y);

    //-- Initial state:
    kalmanFilterCenter.statePre.at<float>(0) = initial_mouse_x; //-- x Position
    kalmanFilterCenter.statePre.at<float>(1) = initial_mouse_y; //-- y Position
    kalmanFilterCenter.statePre.at<float>(2) = 0;		  //-- x Velocity
    kalmanFilterCenter.statePre.at<float>(3) = 0;		  //-. y Velocity

    //-- Set the rest of the matrices:
    cv::setIdentity( kalmanFilterCenter.measurementMatrix );
    cv::setIdentity( kalmanFilterCenter.processNoiseCov, cv::Scalar::all(0.0001));
    cv::setIdentity( kalmanFilterCenter.measurementNoiseCov, cv::Scalar::all(0.1));
    cv::setIdentity( kalmanFilterCenter.errorCovPost, cv::Scalar::all(0.1));
}



//-----------------------------------------------------------------------------------------------------------------------
//-- Refresh the detected hand characteristics
//-----------------------------------------------------------------------------------------------------------------------
void HandDescription::operator ()( const cv::Mat src )
{
    update ( src );
}

void HandDescription::update(const cv::Mat src)
{
    //-- Do things to update each parameter
}



//-----------------------------------------------------------------------------------------------------------------------
//-- Get the characteristics of the hand
//-----------------------------------------------------------------------------------------------------------------------

double HandDescription:: getHandAngle ()
{
    return _hand_angle;
}

std::pair <int, int> HandDescription::getCenterHand ()
{
    return _hand_center;
}

std::vector< std::vector<cv::Point> > HandDescription::getContours()
{
    return _hand_contour;
}

cv::Rect HandDescription::getBoundingBox()
{
    return _hand_bounding_box;
}

int HandDescription::getGesture()
{
    return _hand_gesture;
}

int HandDescription::getNumFingers()
{
    return _hand_num_fingers;
}



//-----------------------------------------------------------------------------------------------------------------------
//-- Plot characteristics on some image
//-----------------------------------------------------------------------------------------------------------------------

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
void HandDescription::contourExtraction(cv::Mat frame, cv::Mat processed)
{
	filteredContour( processed, _hand_contour);

	cv::Mat display; 
	

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
	    
	}
	else
	    std::cerr << "No contours found!" << std::endl; 

	//-----------------------------------------------------------------------------------------------------
	//-- Show processed image
	//-----------------------------------------------------------------------------------------------------
	//cv::imshow( "Processed Stream", display);
}


void HandDescription::angleControl()
{
	//-----------------------------------------------------------------------------------------------------
	//-- Print "Angle control"
	//-----------------------------------------------------------------------------------------------------
	
	//-- Matrix that shows the gauge
	cv::Mat gauge = cv::Mat::zeros( 100, 200, CV_8UC3);
	
	//-- Create matrix for storing the measurement (measured position of hand)
	cv::Mat_<float> angleMeasurement(1, 1);
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
	
	cv::imshow( "Gauge", gauge);
	
}

std::pair <int, int> HandDescription::getCenterHand (cv:: Mat frame)
{
	//-----------------------------------------------------------------------------------------------------
	//-- Move cursor
	//-----------------------------------------------------------------------------------------------------
	
	

    
	//-- Copy the frame to display to draw there the results 
	cv:: Mat display;
	frame.copyTo(display);

	//-- Create matrix for storing the measurement (measured position of hand)
	cv::Mat_<float> measurement(2, 1);
	measurement.setTo( cv::Scalar(0));
    

	if ( (int) _hand_contour.size() > 0 )
	{
		//-- Get image dimensions:
		int imageWidth = frame.cols, imageHeight = frame.rows;
		//std::cout << "Captured image: " << imageWidth << " x " << imageHeight << std::endl;


		//-- Predict next cursor position with kalman filter:
		cv::Mat prediction = kalmanFilterCenter.predict();
		cv::Point predictedPoint( prediction.at<float>(0), prediction.at<float>(1) );

		//-- (Optional) Print predicted point on screen:
		cv::circle( display, predictedPoint, 4, cv::Scalar( 0, 255, 0), 2 );

		//-- Measure actual point (uncomment the selected method):
		int cogX = 0, cogY = 0;

		//-------------------- With RotatedRect --------------------------------------
		/*
		cv::RotatedRect minRect = cv::minAreaRect( _hand_contour[0]);
		cv::Point2f rect_points[4]; minRect.points( rect_points );
		for( int j = 0; j < 4; j++ )
		{
			cogX += rect_points[j].x;
			cogY += rect_points[j].y;
		}

		cogX /= 4;
		cogY /= 4;
		*/

		//-------------------- With Bounding Rectangle --------------------------------
		//-- (This is more stable than the rotated rectangle)
		cv::Rect rect  =  cv::boundingRect(_hand_contour[0]);
		cogX = rect.x + rect.width / 2;
		cogY = rect.y + rect.height / 2;

		measurement(0) = cogX;
		measurement(1) = cogY;

		cv::Point cog( cogX, cogY);

		//-- (Optional) Print cog on screen:
		cv::circle( display, cog, 5, cv::Scalar( 255, 0, 0), 2 );

		//-- Correct estimation:
		cv::Mat estimation = kalmanFilterCenter.correct( measurement);
		cv::Point estimationPoint( estimation.at<float>(0), estimation.at<float>(1) );

		//- (Optional) Print estimation on screen:
		cv::circle( display, estimationPoint, 3, cv::Scalar( 0, 0, 255), 2 );

		//-- Get screen dimensions:
		int screenHeight, screenWidth;
		getDisplayDimensions( screenWidth, screenHeight);

		//-- Get new cursor position by mapping the points:
		int x, y;
		//x = cogX * screenWidth / imageWidth;
		//y = cogY * screenHeight / imageHeight;
		x = estimationPoint.x * screenWidth / imageWidth;
		y = estimationPoint.y * screenHeight / imageHeight;

		return std::pair<int, int> (x, y);
	}

}
