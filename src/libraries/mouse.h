//------------------------------------------------------------------------------
//-- mouse
//------------------------------------------------------------------------------
//--
//-- Functions to control the mouse under linux
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

/*! \file mouse.h
 *  \brief Functions to control the mouse under linux
 *
 * \author David Estevez Fernandez ( http://github.com/David-Estevez )
 * \author Irene Sanz Nieto ( https://github.com/irenesanznieto )
 * \date Dec 12th, 2013
 */

#ifndef MOUSE_H
#define MOUSE_H

#include <iostream>
#include <cstdlib>
#include <unistd.h>

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


//! \brief Sends a click event to the X server
void click();

/*!
 * \brief Get the current screen dimensions
 * \return Current screen dimensions
 */
std::pair<int, int> getDisplayDimensions( );

/*!
 * \brief Get the current mouse position
 * \return Current mouse position
 */
std::pair<int, int> getMousePos();

#endif	//MOUSE_H
