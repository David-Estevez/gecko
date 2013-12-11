#include "HandDetector.h"

//--------------------------------------------------------------------------------------------------------
//-- Constructors
//--------------------------------------------------------------------------------------------------------

HandDetector::HandDetector()
{
    //-- Std deviation multipliers:
    hue_sigma_mult = 6;
    sat_sigma_mult = 6;
    val_sigma_mult = 6;

    //-- Skin color limits
    lower_limit = cv::Scalar( 0, 58, 89);
    upper_limit = cv::Scalar( 25, 173, 229);
    hue_invert = false;

    //-- Initialize cascade classifier:
    initCascadeClassifier();
    initBackgroundSubstractor();
}

HandDetector::HandDetector( cv::Mat& ROI)
{
    //-- Std deviation multipliers:
    hue_sigma_mult = 2;
    sat_sigma_mult = 2;
    val_sigma_mult = 2;

    //-- Skin color limits
    calibrate( ROI );

    //-- Initialize cascade classifier:
    initCascadeClassifier();
    initBackgroundSubstractor();
}



HandDetector::~HandDetector()
{
    cv::destroyWindow("Calibrating skin");
}

//--------------------------------------------------------------------------------------------------------
//-- Calibration functions
//--------------------------------------------------------------------------------------------------------

void HandDetector::defaultValues(cv::VideoCapture cap)
{
    int delay=24;

    cv::Mat default_values=cv::imread("../img/default.png");

    while (1)
    {
        cv::imshow("Default Values", default_values);
        char key =  cv::waitKey(delay);
        if ( key == 10 || key == 13 )
        {
            cv::destroyWindow("Default Values");
            break;
        }
        else if (key==27 || key=='q')
            exit(0);
    }


    HandDetector::calibrationLoop(cap);
}

void HandDetector::customValues(cv::VideoCapture cap)
{

    int delay=24;

    cv::Mat custom=cv::imread("../img/custom.png");

    while (1)
    {
        cv::imshow("CUSTOM VALUES", custom);
        char key =  cv::waitKey(delay);
        if ( key == 10 || key == 13 )
        {
            cv::destroyWindow("CUSTOM VALUES");
            break;
        }
        else if (key==27 || key=='q')
            exit(0);
    }

    bool stop = false;


    while( !stop)
    {
        //-- Get current frame
        cv::Mat frame, cal_screen;
        if (! cap.read( frame ) )
            break;
        cv::flip(frame,frame,1);


        //-- Add calibration frame
        drawCalibrationMarks(frame, cal_screen, halfSide);

        //-- Show calibration screen
        cv::imshow( "CUSTOM VALUES", cal_screen);

        //-- Wait for user confirmation
        char key =  cv::waitKey(delay);
        if ( key == 10 || key == 13 )
        {
            //-- Get region of interest data:
            int image_rows = frame.rows;
            int image_cols = frame.cols;

            cv::Mat ROI = frame( cv::Rect( cv::Point( image_cols / 2 - halfSide,  image_rows/2 - halfSide ),
                           cv::Point( image_cols / 2 + halfSide,  image_rows/2 + halfSide)));
            //cv::imshow( "Test", ROI);

            HandDetector::calibrate( ROI );
            HandDetector::getCalibration( lower, upper);

            //drawHistogramHSV( ROI );

            //-- Close window
            cvDestroyWindow( "CUSTOM VALUES");
            break;
        }
    }
    HandDetector::calibrationLoop(cap);

}

void HandDetector::calibrationLoop(cv::VideoCapture cap)
{
    int delay=24;

    int h[2]={lower_limit[0],upper_limit[0]};
    int s[2]={lower_limit[1],upper_limit[1]};
    int v[2]={lower_limit[2],upper_limit[2]};

    while (1)
    {
        //-- Get current frame
        cv::Mat frame=cv::imread("../img/calibration_loop.png");

        cv::imshow("PRESS ENTER", frame);
        //-- Wait for user confirmation
        char key =  cv::waitKey(delay);
        if ( key == 10 || key == 13 )
        {
            cv::destroyWindow("PRESS ENTER");
            break;
        }
        else if (key==27 || key=='q')
            exit(0);
    }


    cv::namedWindow("Calibrating skin");
    cv::createTrackbar("H min", "Calibrating skin", &h[0], 255);
    cv::createTrackbar("H max", "Calibrating skin", &h[1], 255);

    cv::createTrackbar("S min", "Calibrating skin", &s[0], 255);
    cv::createTrackbar("S max", "Calibrating skin", &s[1], 255);

    cv::createTrackbar("V min", "Calibrating skin", &v[0], 255);
    cv::createTrackbar("V max", "Calibrating skin", &v[1], 255);

    while (1)
    {
        //-- Get current frame
        cv::Mat frame, dst;
        if (! cap.read( frame ) )
            break;
        cv::flip(frame,frame,1);

        //change the skin thresholding limits using the trackbars
        lower_limit=cv::Scalar(cv::getTrackbarPos("H min", "Calibrating skin"), cv::getTrackbarPos("S min", "Calibrating skin"), cv::getTrackbarPos("V min", "Calibrating skin"));
        upper_limit=cv::Scalar(cv::getTrackbarPos("H max", "Calibrating skin"), cv::getTrackbarPos("S max", "Calibrating skin"), cv::getTrackbarPos("V max", "Calibrating skin"));
        HandDetector::filter_hand(frame, dst);

        cv::imshow("Calibrating skin", dst);
        //-- Wait for user confirmation
        char key =  cv::waitKey(delay);
        if ( key == 10 || key == 13 )
        {
            cv::destroyWindow("Calibrating skin");
            break;
        }
        else if (key==27 || key=='q')
            exit(0);

    }

}


void HandDetector::calibrate(cv::Mat &ROI)
{
    //-- Convert from BGR to HSV
    cv::Mat HSV_ROI;
    cv::cvtColor( ROI, HSV_ROI, CV_BGR2HSV);

    //-- Split HSV ROI in channels:
    std::vector<cv::Mat> HSV_splitted;
    cv::split( HSV_ROI, HSV_splitted);

    //-- Calculate values:
    int hue_mean = median( HSV_splitted[0] );
    //int sat_mean = median( HSV_splitted[1] );
    //int val_mean = median( HSV_splitted[2] );

    int hue_sigma = stdDeviation( HSV_splitted[0] );
    //int sat_sigma = stdDeviation( HSV_splitted[1] );
    //int val_sigma = stdDeviation( HSV_splitted[2] );

    //-- Calculate limits
    int hue_lower_limit = hue_mean - hue_sigma_mult * hue_sigma / 2;
    //int sat_lower_limit = sat_mean - sat_sigma_mult * sat_sigma / 2;
    //int val_lower_limit = val_mean - val_sigma_mult * val_sigma / 2;

    int hue_upper_limit = hue_mean + hue_sigma_mult * hue_sigma / 2;
    //int sat_upper_limit = sat_mean + sat_sigma_mult * sat_sigma / 2;
    //int val_upper_limit = val_mean + val_sigma_mult * val_sigma / 2;

    //-- Check the limits of the values
    if ( hue_lower_limit < 0)
    {
	int aux = hue_lower_limit;
	hue_lower_limit = hue_upper_limit;
	hue_upper_limit = 180 - abs(aux);
	hue_invert = true;
    }
    else if ( hue_upper_limit > 179)
    {
	int aux = hue_upper_limit;
	hue_upper_limit = hue_lower_limit;
	hue_lower_limit = aux - 180;
	hue_invert = true;
    }
    else
	hue_invert = false;
/*
    if ( sat_lower_limit < 0) sat_lower_limit = 0;
    if ( val_lower_limit < 0) val_lower_limit = 0;
    if ( sat_upper_limit > 255 ) sat_upper_limit = 255;
    if ( val_lower_limit > 255 ) val_upper_limit = 255;
*/
    //-- Compose the values:
  //  lower_limit = cv::Scalar( hue_lower_limit, sat_lower_limit, val_lower_limit );
  //  upper_limit = cv::Scalar( hue_upper_limit, sat_upper_limit, val_upper_limit );

    lower_limit = cv::Scalar( hue_lower_limit, 58, 89  );
    upper_limit = cv::Scalar( hue_upper_limit, 173, 229 );

    std::cout << "[Debug] Lower limit is: " << lower_limit << std::endl;
    std::cout << "[Debug] Upper limit is: " << upper_limit << std::endl;
    std::cout << "[Debug] Inverting hue: " << hue_invert << std::endl;
}


void HandDetector::calibrate(cv::Scalar lower_limit, cv::Scalar upper_limit)
{
    this->lower_limit = lower_limit;
    this->upper_limit = upper_limit;
    this->hue_invert = false;
}

void HandDetector::getCalibration(cv::Scalar &lower_limit, cv::Scalar &upper_limit)
{
    lower_limit = this->lower_limit;
    upper_limit = this->upper_limit;
}


//--------------------------------------------------------------------------------------------------------
//-- Hand-detection
//--------------------------------------------------------------------------------------------------------

void HandDetector::operator ()(cv::Mat& src, cv::Mat& dst)
{
    filter_hand( src, dst);
    //-- Show result (optional)
    //------------------------------------------------

    if(!dst.empty())
        cv::imshow("Skin Threshold", dst );
}


void HandDetector::filter_hand(cv::Mat &src, cv::Mat &dst)
{
    static int it=0;


    //-- Filter out head:
    //------------------------------------------------
    //-- Get mask
    cv::Mat headTrackingMask;
    filterFace( src, headTrackingMask );
//    cv::imshow("[Debug] Face", src);

    //-- Background substraction:
    //------------------------------------------------
    cv::Mat withoutBackground;
    backgroundSubstraction( src, withoutBackground );

    //-- Skin thresholding
    //------------------------------------------------
    cv::Mat thresholdedHand;
    threshold( withoutBackground, thresholdedHand );

    //-- Filter out small blobs:
    //------------------------------------------------
    cv::Mat withoutBlobs;
    filterBlobs( thresholdedHand, withoutBlobs );

    cv::Mat dummy;
    cv::bitwise_and( withoutBlobs, headTrackingMask, dummy );

    static cv::Mat sum=cv::Mat::zeros(dummy.rows, dummy.cols,dummy.type());
    static cv::Mat last_sum=cv::Mat::zeros(dummy.rows, dummy.cols,dummy.type());


    if (it <3)
    {
        sum+=dummy;
        it++;

    }
    else
    {
        it=0;
        sum.copyTo(last_sum);
        dummy.copyTo(sum);

    }

    last_sum.copyTo(dst);

    //    std::cerr<<"Iterator number: "<<it<<std::endl;




}



//--------------------------------------------------------------------------------------------------------
//-- Face detection
//--------------------------------------------------------------------------------------------------------

//-- Initialize the cascade classifier:
void HandDetector::initCascadeClassifier( )
{
    //-- Load file with the classifier features:
    if ( ! faceDetector.load( "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml" ) )
    {
	std::cerr << "[Error] Could not load cascade classifier features file." << std::endl;
    }

    //-- Factors to resize the face-detection
    factorX = 1.25;
    factorY = 1.25;

}

//-- Filter out faces:
void HandDetector::filterFace(const cv::Mat &src, cv::Mat &dstMask )
{
    //-- Create variables to store faces
    std::vector< cv::Rect > detectedFaces;

    //-- Convert the source image to a greyscale image:
    cv::Mat srcGrey;
    cv::cvtColor( src, srcGrey, CV_BGR2GRAY );

    cv::Mat srcGreySmall;
    cv::resize( srcGrey, srcGreySmall, cv::Size(0, 0), 0.25, 0.25 );

    //-- Detect face:
    faceDetector.detectMultiScale( srcGreySmall, detectedFaces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size( 30, 30) );

    dstMask = cv::Mat(  srcGrey.size() , CV_8UC1,  cv::Scalar( 255, 255, 255) );

    //-- Clear detected faces:
    lastFacesPos.clear();

    //-- Show detected faces:
    if ( ! detectedFaces.empty() )
    {
	for (int i = 0; i < detectedFaces.size(); i++)
	{
	    cv::Rect resizedRect;

	    if ( factorX == 1 && factorY == 1 )
	    {
		//-- Do not resize:
		resizedRect = cv::Rect( detectedFaces[i].x * 4,
					detectedFaces[i].y * 4,
					detectedFaces[i].width  * 4,
					detectedFaces[i].height * 4 );
	    }
	    else
	    {
		//-- Resize:
		//-- Calculate original center:
		double cx = detectedFaces[i].x * 4 + detectedFaces[i].width * 2;
		double cy = detectedFaces[i].y * 4 + detectedFaces[i].height * 2;

		//-- Calculate new size:
		double newW = detectedFaces[i].width * 4 * factorX;
		double newH = detectedFaces[i].width * 4 * factorY;

		//-- Calculate new center:
		double newX = cx - newW / 2;
		double newY = cy - newH / 2;

		resizedRect = cv::Rect( newX, newY, newW, newH );
	    }

	    cv::rectangle( dstMask, resizedRect, cv::Scalar( 0, 0, 0), CV_FILLED );

	    //-- Save rectancle
	    lastFacesPos.push_back( resizedRect );
	}
    }
}


//-- Return last faces found
std::vector< cv::Rect >& HandDetector::getLastFacesPos()
{
    return lastFacesPos;
}


//-- Plot the last faces found marks
void HandDetector::drawFaceMarks(const cv::Mat &src, cv::Mat &dst, cv::Scalar color, int thickness )
{
    //-- Allocate the dst matrix if empty:
    if ( dst.total() == 0)
	    dst = src.clone();

    //-- Plot the bounding rectangles:
    if ( !lastFacesPos.empty() )
    {
	std::cout << "[Debug] Detected " << lastFacesPos.size() << " face(s)." << std::endl;
	for ( int i = 0; i < lastFacesPos.size(); i++)
	    cv::rectangle( dst, lastFacesPos[i], color, thickness );
    }

}

cv::Scalar HandDetector::getLower()
{
	return lower;
}
cv::Scalar HandDetector:: getUpper()
{
	return upper;
}



//--------------------------------------------------------------------------------------------------------
//-- Statistical functions:
//--------------------------------------------------------------------------------------------------------

int HandDetector::average(cv::Mat &ROI)
{
    //-- Calculate average value
    double average = 0;

    for (int i = 0; i < ROI.cols; i++ )
	for( int j = 0; j < ROI.rows; j++)
	    average += (int) ROI.at<unsigned char>(i, j);


    average /= (double) (ROI.cols * ROI.rows);
    return ceil(average);
}

int HandDetector::median(cv::Mat &ROI)
{
    //-- Unroll the ROI
    std::vector<unsigned char> unrolled;

    for (int i = 0; i < ROI.cols; i++ )
	for( int j = 0; j < ROI.rows; j++)
	     unrolled.push_back( ROI.at<unsigned char>(i, j) );

    //-- Sort the vector
    std::sort( unrolled.begin(), unrolled.end() );

    //-- Get the median
    if ( unrolled.size() % 2 != 0)
    {
	//-- Odd vector:
	return ceil( unrolled.at( (unrolled.size() -1 ) / 2) );
    }
    else
    {
	//-- Even vector:
	double mean = unrolled.at( unrolled.size() / 2 ) + unrolled.at( (unrolled.size() / 2 ) - 1 );
	mean /= 2;
	return ceil(mean);
    }
}

int HandDetector::stdDeviation(cv::Mat &ROI)
{
    //-- Calculates the std deviation of the pixel values
    double stddev = 0;

    for (int i = 0; i < ROI.cols; i++ )
	for( int j = 0; j < ROI.rows; j++)
	    stddev +=  pow( (int) ROI.at<unsigned char>(i, j), 2);

    stddev = sqrt( stddev ) / (ROI.cols * ROI.rows);
    return ceil(stddev);
}


//-------------------------------------------------------------------------------------------------------------
//-- Hand-filtering functions
//-------------------------------------------------------------------------------------------------------------
void HandDetector::initBackgroundSubstractor()
{
    bg.set("nmixtures",3);// set number of gaussian mixtures
    bg.set("detectShadows", true); //if false: turn shadow detection off
    bg.setbackgroundRatio(0.000000000000001);

}

void HandDetector::backgroundSubstraction(cv::Mat &src, cv::Mat &dst)
{
    dst = src.clone();
    backgroundSubs(dst, bg);
}


void HandDetector::threshold(const cv::Mat &src, cv::Mat &dst)
{
    //-- Convert to HSV
    cv::Mat hsv;
    cv::cvtColor( src, hsv, CV_BGR2HSV);

    //-- Threshold
    cv::inRange(hsv, lower_limit, upper_limit, dst);

    //-- If color limit is arround 0, hue channel needs to be inverted
    if (hue_invert)
    {
		std::vector< cv::Mat > hsv;
		cv::split( dst, hsv);
		cv::bitwise_not( hsv[0], hsv[0] );
		//! \todo Join them again?
    }
}

void HandDetector::filterBlobs(const cv::Mat &src, cv::Mat &dst)
{
//    cv::Mat kernel = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 5, 5) );
//    cv::morphologyEx( src, dst, cv::MORPH_CLOSE, kernel);

    cv::GaussianBlur(src, dst,cv::Size(0,0),5);
//    cv::adaptiveThreshold(dst, dst, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, 5);

//    cv::threshold(dst, dst, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);


    int min=50;
    cv::inRange(dst,cv::Scalar(min,min,min) , cv::Scalar(255,255,255), dst);

}


