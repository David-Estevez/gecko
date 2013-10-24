//--------------------------------------------------------------------------------------
//-- mouse.h
//--------------------------------------------------------------------------------------
//-- Functions to control the mouse
//--------------------------------------------------------------------------------------
#ifndef mouse_h
#define mouse_h

#include <iostream>
#include <cstdlib>

//-- Include libraries to control the mouse in Linux
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>

void moveMouse( const int& x, const int& y, const bool absoluteMode = true);
void getMousePos( int &x, int &y);
void getDisplayDimensions( int& width, int& height);

#endif
