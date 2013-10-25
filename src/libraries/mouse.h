//--------------------------------------------------------------------------------------
//-- mouse.h
//--------------------------------------------------------------------------------------
//-- Functions to control the mouse
//--------------------------------------------------------------------------------------
#ifndef MOUSE_H
#define MOUSE_H

#include <iostream>
#include <cstdlib>

#include <opencv2/opencv.hpp>

//-- Include libraries to control the mouse in Linux
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>

class Mouse 
{
	public:
		Mouse();
		void moveMouse( const int& x, const int& y, const bool absoluteMode = true);
		void getMousePos( int &x, int &y);
		void getDisplayDimensions( int& width, int& height);
		void moveCursor (cv:: Mat, std::vector< std::vector<cv::Point> >);
		
	private: 
		cv::KalmanFilter kalmanFilter;

};

#endif	//MOUSE_H
