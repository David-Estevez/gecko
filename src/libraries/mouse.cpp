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

/*! \file mouse.cpp
 *  \brief Functions to control the mouse under linux
 *
 * \author David Estevez Fernandez ( http://github.com/David-Estevez )
 * \author Irene Sanz Nieto ( https://github.com/irenesanznieto )
 * \date Dec 12th, 2013
 */

#include "mouse.h"

void moveMouse(std::pair <int, int> coordinates, const bool absoluteMode)
{
    Display * displayMain = XOpenDisplay( NULL);

    if ( displayMain == NULL)
    {
	std::cerr << "Display opening error." << std::endl;
	exit( EXIT_FAILURE);
    }

    if (absoluteMode)
    {
	std::pair <int, int> current = getMousePos();

	int xDisplacement = coordinates.first - current.first;
	int yDisplacement = coordinates.second - current.second;

	XWarpPointer( displayMain, None, None, 0, 0, 0, 0, xDisplacement, yDisplacement);
    }
    else
    {
	//-- Relative movement:
	XWarpPointer( displayMain, None, None, 0, 0, 0, 0, coordinates.first, coordinates.second);
    }
    XCloseDisplay( displayMain);
}

void moveMousePercentage(std::pair<double, double> coordinates)
{
    //-- Get screen dimensions
    std::pair< int, int> screen = getDisplayDimensions();

    //-- Calculate the new position
    std::pair< int, int> newPosition;
    newPosition.first = (int) ( coordinates.first * screen.first);
    newPosition.second = (int) ( coordinates.second * screen.second);

    //-- Move there
    moveMouse( newPosition, true);
}

std::pair<int, int> getDisplayDimensions( )
{
    Display * displayMain = XOpenDisplay( NULL);

    int width  = XDisplayWidth(  displayMain, 0);
    int height = XDisplayHeight( displayMain, 0);

    XCloseDisplay( displayMain);

    return std::pair<int, int> ( width, height );
}

std::pair<int, int> getMousePos()
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

	return std::pair <int, int> ( root_x, root_y );
}




void click()
{
    //-- Open a display of X server:
    Display *display = XOpenDisplay(NULL);
    XEvent event;

    if ( display == NULL)
    {
        std::cerr << "[Mouse] Error: error opening display." << std::endl;
        exit(-1);
    }

    //-- Setup event to send:
    memset( &event, 0x00, sizeof(event));
    event.type = ButtonPress;
    event.xbutton.button = Button1;
    event.xbutton.same_screen = True;

    XQueryPointer( display, RootWindow(display, DefaultScreen(display)), &event.xbutton.root, &event.xbutton.window, &event.xbutton.x_root, &event.xbutton.y_root, &event.xbutton.x, &event.xbutton.y, &event.xbutton.state);

    event.xbutton.subwindow = event.xbutton.window;

    while(event.xbutton.subwindow)
    {
        event.xbutton.window = event.xbutton.subwindow;
        XQueryPointer(display, event.xbutton.window, &event.xbutton.root, &event.xbutton.subwindow, &event.xbutton.x_root, &event.xbutton.y_root, &event.xbutton.x, &event.xbutton.y, &event.xbutton.state);
    }

    if(XSendEvent(display, PointerWindow, True, 0xfff, &event) == 0)
        std::cerr << "[Mouse] Error: error ocurred while sending the click event." << std::endl;

    XFlush(display);

    usleep(100000);

    event.type = ButtonRelease;
    event.xbutton.state = 0x100;

    if(XSendEvent(display, PointerWindow, True, 0xfff, &event) == 0)
        std::cerr << "[Mouse] Error: error ocurred while sending the click release event." << std::endl;

    XFlush(display);

    XCloseDisplay(display);

    std::cout << "[Debug] Click! " << std::endl;

}
