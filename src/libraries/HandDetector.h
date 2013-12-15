//------------------------------------------------------------------------------
//-- HandDetector
//------------------------------------------------------------------------------
//--
//-- Extracts a binary image with the silouette of the candidates to be a hand
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

/*! \file HandDetector.h
 *  \brief Extracts a binary image with the silouette of the candidates to be a hand
 *
 * \author David Estevez Fernandez ( http://github.com/David-Estevez )
 * \author Irene Sanz Nieto ( https://github.com/irenesanznieto )
 * \date Dec 12th, 2013
 */


#ifndef HAND_DETECTOR
#define HAND_DETECTOR

#include <opencv2/opencv.hpp>
#include "handUtils.h"


/*! \class HandDetector
 *  \brief Segments the hand silouette from the original image and returns the information in a binary image
 *
 *
 *  This class offers different means of calibrating the skin:
 *   -Theoretical HSV values
 *   -Custom HSV values calculated from a sample of the user's skin color
 *
 *  In order to obtain the segmented hand, the operator () or the function filter_hand may be used
 */
class HandDetector
{

public:

	//-- Constructors
    //-----------------------------------------------------------------------
    //! \brief Default constructor
    HandDetector();
    //! \brief Constructor that takes as argument an image of the hand's skin to obtain the custom HSV range
	HandDetector( cv::Mat& ROI);

    //-- Destructor
    //-----------------------------------------------------------------------
    ~HandDetector();


	//-- Calibration loop
    //! \brief Allows the user to adjust the HSV range manually to improve the segmentation
    void calibrationLoop(cv::VideoCapture);
    //! \brief Shows an image with information and starts calibrationLoop function
    void defaultValues(cv::VideoCapture cap);
    /*! \brief Obtains the custom HSV skin range
     *
     *  Shows and image with information, then shows the calibration image and from the information obtained calculates
     *  the HSV skin color range. Finally, it calls the calibrationLoop function
     */
    void customValues(cv::VideoCapture cap);


	//-- Calibration functions
    //-----------------------------------------------------------------------

    //! \brief Changes the HSV range accordingly with the input skin color image
	void calibrate( cv::Mat& ROI);

    //! \brief Sets the HSV skin colors range with the inputs. If there are no inputs, default values are set.
	void calibrate( cv::Scalar lower_limit = cv::Scalar( 0, 58, 89), cv::Scalar upper_limit = cv::Scalar( 25, 173, 229) );
    //! \brief Returns the HSV range
    void getCalibration( cv::Scalar& lower_limit, cv::Scalar& upper_limit);

	//-- Hand-detection
    //-----------------------------------------------------------------------
    /*! \brief Update the segmented hand binary image
     *
     *  This operator is a wrapper of the filter_hand function.
     *
     *  \param src Original image coming from the video input.
     *  \param dst Final binary image containing the segmented image.
     */
    void operator()(cv::Mat& src, cv::Mat& dst);

    /*! \brief Update the segmented hand image using the new frame of the video input.
     *
     *  Removes the background, thresholds the skin color and makes morphology transformations to improve the binary output image
     *
     *  \param src Original image coming from the video input.
     *  \param dst Final binary image containing the segmented image.
     */
    void filter_hand(cv::Mat& src, cv::Mat& dst);

	//-- Face-tracking
    //-----------------------------------------------------------------------
    //! \brief Returns the last position of the face
	std::vector< cv::Rect >& getLastFacesPos();

    /*! \brief Tracks and covers the faces that appear in the image.
     *
     *  The faces are covered with a square so they do not interfere with the rest of the segmentation.
     *
     *  \param src Original image coming from the video input.
     *  \param dst Output image with the squares over the faces.
     *  \param color Color of the squares, default is black.
     *  \param thickness Thickness of the square drawn over the faces.
     */
	void drawFaceMarks( const cv::Mat& src, cv::Mat& dst , cv::Scalar color = cv::Scalar(0, 255, 0), int thickness = 1  );
	
    //-- Get lower and upper level (calibration)
    //-----------------------------------------------------------------------
    //! \brief Returns the lower HSV skin values
    cv::Scalar getLower();
    //! \brief Returns the upper HSV skin values
    cv::Scalar getUpper();


    private:
	//-- Statistical functions:
    //! \todo OpenCV already has this functions, so use them instead.
    //! \brief Returns the average of the pixel's values
	int average( cv::Mat& ROI);
    //! \brief Returns the median of the pixel's values
	int median( cv::Mat& ROI);
    //! \brief Returns the standard deviation of the pixel's values
	int stdDeviation( cv::Mat& ROI);

	//-- Hand filtering functions:
    //-----------------------------------------------------------------------
    /*! \brief Thresholds the input image using the HSV range
     *
     *  \param src Input image
     *  \param dst Binary output image
     */
	void threshold( const cv::Mat& src, cv::Mat& dst);
    /*! \brief Applies Gaussian Blur and thresholding to improve the final binary image
     *
     *  \param src Input image
     *  \param dst Binary output image
     */
	void filterBlobs( const cv::Mat& src, cv::Mat& dst);

	//-- Filter contours:
	void filterContours( std::vector< std::vector < cv::Point > >& contours , std::vector< std::vector < cv::Point > >& filteredContours);

    //-- Background substractor:
    //---------------------------------------------------------------------------------
    //! \brief Background Subtractor object that derives from cv::BackgroundSubtractorMOG2
    backgroundSubstractor bg ;
    //! \brief Sets the parameters of the background subtractor object
    void initBackgroundSubstractor();
    /*! \brief Substracts the background from the input image
     *
     *  \param src Input image
     *  \param dst Output image
     */
    void backgroundSubstraction(cv::Mat& src, cv::Mat& dst);


	//-- Filter face:
	//----------------------------------------------------------------------------------
	//! -- \brief Cascade classifier to detect faces:
	cv::CascadeClassifier faceDetector;

	//! -- \brief Position of the last faces found:
	std::vector< cv::Rect > lastFacesPos;

	//! -- \brief Resize the rectangles.
	double factorX, factorY;

	//! -- \brief Initializes the face detector
	void initCascadeClassifier();

	//! -- \brief Removes the face from the src image
	void filterFace(const cv::Mat& src, cv::Mat& dstMask );



	//-- Skin hue calibration
	//-----------------------------------------------------------------------------------
	//-- HSV limits
    //! \brief Lower limit of the HSV skin range
	cv::Scalar lower_limit;
    //! \brief Upper limit of the HSV skin range
	cv::Scalar upper_limit;
    //! \brief Boolean that will be true if color limit is arround 0
	bool hue_invert;

	//-- HSV sigma multiplier
    //! \brief Hue sigma multiplier used when calculating the custom HSV skin range
	int hue_sigma_mult;
    //! \brief Saturation sigma multiplier used when calculating the custom HSV skin range
	int sat_sigma_mult;
    //! \brief Value sigma multiplier used when calculating the custom HSV skin range
	int val_sigma_mult;

    //! \brief Size of the calibration box used when capturing the custom HSV range
    static const int halfSide=40;

    //! \brief Lower limit of the HSV skin range
    cv::Scalar lower;
    //! \brief Upper limit of the HSV skin range
    cv::Scalar upper;
};

#endif // HAND_DETECTOR
