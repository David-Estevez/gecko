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

/*!
 *  \brief Moves the cursor to the specified position
 *
 *  When absoluteMode is true, the coordinates given refer to the
 *  absolute position of the screen, otherwise, coordinates specify
 *  the increment in x and y that the cursor has to move.
 *
 */
void moveMouse( std::pair <int, int> coordinates, const bool absoluteMode = true);

/*!
 * \brief Moves the cursor mapping a percentage into the current screen resolution
 *
 * \param coordinates Pair of values between 0 and 1 specifying the position as a fraction
 * of the current screen resolution. (0.5, 0.5) would refer to the center of the screen.
 *
 */
void moveMousePercentage( std::pair <double, double> coordinates );


std::pair<int, int> getDisplayDimensions( );
std::pair<int, int> getMousePos();

#endif	//MOUSE_H
