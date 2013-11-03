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


