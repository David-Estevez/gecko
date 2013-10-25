#include "mouse.h"

Mouse::Mouse()
{
	//-- Kalman filter mouse setup
    //---------------------------------------------------------------------

    //-- Create filter:
    kalmanFilter.init( 4, 2, 0);
    kalmanFilter.transitionMatrix = *( cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
								0, 1, 0, 1,
								0, 0, 1, 0,
								0, 0, 0, 1);

    //-- Create matrix for storing the measurement (measured position of hand)
    cv::Mat_<float> measurement(2, 1);
    measurement.setTo( cv::Scalar(0));

    //-- Get mouse position:
    //! \todo Change this for screen center?
    int initial_mouse_x, initial_mouse_y;
    getMousePos( initial_mouse_x, initial_mouse_y);

    //-- Initial state:
    kalmanFilter.statePre.at<float>(0) = initial_mouse_x; //-- x Position
    kalmanFilter.statePre.at<float>(1) = initial_mouse_y; //-- y Position
    kalmanFilter.statePre.at<float>(2) = 0;		  //-- x Velocity
    kalmanFilter.statePre.at<float>(3) = 0;		  //-. y Velocity

    //-- Set the rest of the matrices:
    cv::setIdentity( kalmanFilter.measurementMatrix );
    cv::setIdentity( kalmanFilter.processNoiseCov, cv::Scalar::all(0.0001));
    cv::setIdentity( kalmanFilter.measurementNoiseCov, cv::Scalar::all(0.1));
    cv::setIdentity( kalmanFilter.errorCovPost, cv::Scalar::all(0.1));
}

void Mouse::controlMouse ()
{}

void Mouse::moveMouse(const int &x, const int &y, const bool absoluteMode)
{
    Display * displayMain = XOpenDisplay( NULL);

    if ( displayMain == NULL)
    {
	std::cerr << "Display opening error." << std::endl;
	exit( EXIT_FAILURE);
    }

    if (absoluteMode)
    {
	int currentX, currentY;
	getMousePos( currentX, currentY);

	int xDisplacement = x - currentX;
	int yDisplacement = y - currentY;

	XWarpPointer( displayMain, None, None, 0, 0, 0, 0, xDisplacement, yDisplacement);
    }
    else
    {
	//-- Relative movement:
	XWarpPointer( displayMain, None, None, 0, 0, 0, 0, x, y);
    }
    XCloseDisplay( displayMain);
}

void Mouse::getDisplayDimensions( int& width, int& height)
{
    Display * displayMain = XOpenDisplay( NULL);

    width = XDisplayWidth( displayMain, 0);
    height= XDisplayHeight( displayMain, 0);

    XCloseDisplay( displayMain);
}

void Mouse::getMousePos( int &x, int &y)
{
	Bool result;

	Window *root_windows;
	Window window_returned;

	int root_x, root_y;
	int win_x, win_y;
	unsigned int mask_return;

	Display *display = XOpenDisplay(NULL);
	if ( display == NULL)
	{
	    std::cerr << "Display opening error." << std::endl;
	    exit( EXIT_FAILURE);
	}

	int number_of_screens = XScreenCount(display);
	//std::cout << "There are " << number_of_screens << " screens." << std::endl;


	root_windows = (Window *) malloc(sizeof(Window) * number_of_screens);

	for (int i = 0; i < number_of_screens; i++)
	{
	    root_windows[i] = XRootWindow(display, i);
	}

	for (int i = 0; i < number_of_screens; i++)
	{
	    result = XQueryPointer(display, root_windows[i], &window_returned,
		    &window_returned, &root_x, &root_y, &win_x, &win_y,
		    &mask_return);
	    if (result == True) {
		break;
	    }
	}
	if (result != True)
	{
	   std::cerr <<  "No mouse found." << std::endl;
	   exit(-1);
	}

	//std::cout << "Mouse is at (" << root_x << ", " << root_y << ")" << std::endl;

	free(root_windows);
	XCloseDisplay(display);

	x = root_x;
	y = root_y;
}

void Mouse::moveCursor (cv:: Mat frame)
{

    
	//-----------------------------------------------------------------------------------------------------
	//-- Move cursor
	//-----------------------------------------------------------------------------------------------------

//	    if ( (int) handContour.size() > 0 )
//	    {
//		//-- Get image dimensions:
//		int imageWidth = frame.cols, imageHeight = frame.rows;
//		//std::cout << "Captured image: " << imageWidth << " x " << imageHeight << std::endl;


//		//-- Predict next cursor position with kalman filter:
//		cv::Mat prediction = kalmanFilter.predict();
//		cv::Point predictedPoint( prediction.at<float>(0), prediction.at<float>(1) );

//		//-- (Optional) Print predicted point on screen:
//		cv::circle( display, predictedPoint, 4, cv::Scalar( 0, 255, 0), 2 );

//		//-- Measure actual point (uncomment the selected method):
//		int cogX = 0, cogY = 0;

//		//-------------------- With RotatedRect --------------------------------------
//		/*
//		cv::RotatedRect minRect = cv::minAreaRect( handContour[0]);
//		cv::Point2f rect_points[4]; minRect.points( rect_points );
//		for( int j = 0; j < 4; j++ )
//		{
//		    cogX += rect_points[j].x;
//		    cogY += rect_points[j].y;
//		}

//		cogX /= 4;
//		cogY /= 4;
//		*/

//		//-------------------- With Bounding Rectangle --------------------------------
//		//-- (This is more stable than the rotated rectangle)
//		cv::Rect rect  =  cv::boundingRect(handContour[0]);
//		cogX = rect.x + rect.width / 2;
//		cogY = rect.y + rect.height / 2;

//		measurement(0) = cogX;
//		measurement(1) = cogY;

//		cv::Point cog( cogX, cogY);

//		//-- (Optional) Print cog on screen:
//		cv::circle( display, cog, 5, cv::Scalar( 255, 0, 0), 2 );

//		//-- Correct estimation:
//		cv::Mat estimation = kalmanFilter.correct( measurement);
//		cv::Point estimationPoint( estimation.at<float>(0), estimation.at<float>(1) );

//		//- (Optional) Print estimation on screen:
//		cv::circle( display, estimationPoint, 3, cv::Scalar( 0, 0, 255), 2 );

//		//-- Get screen dimensions:
//		int screenHeight, screenWidth;
//		getDisplayDimensions( screenWidth, screenHeight);

//		//-- Get new cursor position by mapping the points:
//		int x, y;
//		//x = cogX * screenWidth / imageWidth;
//		//y = cogY * screenHeight / imageHeight;
//		x = estimationPoint.x * screenWidth / imageWidth;
//		y = estimationPoint.y * screenHeight / imageHeight;

//		//-- Move the mouse to the specified position:
//		moveMouse( x, y );
//	    }
//	

}
