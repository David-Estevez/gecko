#include "mouse.h"

void moveMouse(const int &x, const int &y, const bool absoluteMode)
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

void getDisplayDimensions( int& width, int& height)
{
    Display * displayMain = XOpenDisplay( NULL);

    width = XDisplayWidth( displayMain, 0);
    height= XDisplayHeight( displayMain, 0);

    XCloseDisplay( displayMain);
}

void getMousePos( int &x, int &y)
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
