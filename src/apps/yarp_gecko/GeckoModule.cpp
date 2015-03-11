#include "GeckoModule.hpp"


const float gecko::GeckoModule::MODULE_PERIOD = 3.0;


const std::string gecko::GeckoModule::PORT_PREFIX = "/gecko";
const std::string gecko::GeckoModule::INPUT_PORT = "/src:i";
const std::string gecko::GeckoModule::DEBUG_PORT = "/debug:o";
const std::string gecko::GeckoModule::SEGMENTATION_DEGUB_PORT = "/segmentation:o";
const std::string gecko::GeckoModule::GESTURE_PORT = "/gesture:o";
const std::string gecko::GeckoModule::POSITION_PORT = "/handPos:o";


gecko::GeckoModule::GeckoModule()
{
    debugOn = false;
    segmentationDebugOn = false;
}

bool gecko::GeckoModule::configure(yarp::os::ResourceFinder &rf)
{
    //--Help
    if (rf.check("help"))
    {
        //-- Show help
    }

    //-- This enables image output on debug port
    debugOn = rf.check("debug");

    //-- This enables segmentation image output on debug port
    segmentationDebugOn = rf.check("debugSegmentation");

    //-- This auto-connects the input stream with the specified port
    connectInput = rf.check("connectInput");
    if (connectInput)
    {
         rgbStreamPort = rf.find("connectInput").asString();
         if (rgbStreamPort.compare("") == 0)
         {
             CD_ERROR("Cannot connect to empty string port\n");
             connectInput = false;
         }
    }

    return openPorts();
}

double gecko::GeckoModule::getPeriod()
{
    return MODULE_PERIOD;
}

bool gecko::GeckoModule::updateModule()
{

}

void gecko::GeckoModule::onRead(gecko::Image &src)
{
    CD_INFO("Received image!\n");

    //-- Extract OpenCV image from YARP image
    IplImage *cvImage = cvCreateImage(cvSize(src.width(), src.height()), IPL_DEPTH_8U, 3 );
    cvCvtColor((IplImage*)src.getIplImage(), cvImage, CV_RGB2BGR);

    cv::Mat frame(cvImage);

    //-- Hand segmentation
    cv::Mat processed;
    handDetector(frame, processed);

    //-- Send back segmentation image if debug is enabled
    if (segmentationDebugOn)
    {
        cv::Mat display = processed.clone();
        IplImage display_ipl = IplImage(display);

        Image& out = segmentation_debug_port.prepare();
        out.zero();
        out.wrapIplImage(&display_ipl);
        segmentation_debug_port.write();
    }

    //-- Contour extraction
    handDescriptor( processed );

    //-- Send back image if debug is enabled
    if (debugOn)
    {
        cv::Mat display = frame.clone();
        handDescriptor.plotHandInterface(display, display);
        IplImage display_ipl = IplImage(display);

        Image& out = debug_port.prepare();
        out.zero();
        out.wrapIplImage(&display_ipl);
        debug_port.write();
    }


}

bool gecko::GeckoModule::close()
{
    return closePorts();
}

bool gecko::GeckoModule::openPorts()
{
    //-- Input port
    if (!input_port.open(PORT_PREFIX+INPUT_PORT))
    {
        CD_ERROR("Could not open input port at %s\n", (PORT_PREFIX+INPUT_PORT).c_str());
        return false;
    }
    else
    {
        //! \todo Enable callback here!!!
        input_port.useCallback(*this);
    }

    //-- Gesture port
    if (!gesture_port.open(PORT_PREFIX+GESTURE_PORT))
    {
        CD_ERROR("Could not open gesture output port at %s\n", (PORT_PREFIX+GESTURE_PORT).c_str());
        return false;
    }

    //-- Position port
    if (!position_port.open(PORT_PREFIX+POSITION_PORT))
    {
        CD_ERROR("Could not open position output port at %s\n", (PORT_PREFIX+POSITION_PORT).c_str());
        return false;
    }

    //-- Debug port
    if ( debugOn )
    {
        if (!debug_port.open(PORT_PREFIX+DEBUG_PORT))
        {
            CD_ERROR("Could not open debug output port at %s\n", (PORT_PREFIX+DEBUG_PORT).c_str());
            return false;
        }
    }

    //-- Debug port
    if ( segmentationDebugOn )
    {
        if (!segmentation_debug_port.open(PORT_PREFIX+SEGMENTATION_DEGUB_PORT))
        {
            CD_ERROR("Could not open debug output port at %s\n", (PORT_PREFIX+SEGMENTATION_DEGUB_PORT).c_str());
            return false;
        }
    }

    //--Optional connect for input port
    if (connectInput)
    {
        if(!yarp::os::Network::connect(rgbStreamPort, PORT_PREFIX+INPUT_PORT))
        {
            CD_ERROR("Could not connect input port at %s with port %s\n", (PORT_PREFIX+INPUT_PORT).c_str(), rgbStreamPort.c_str());
            return false;
        }
    }

    return true;
}

bool gecko::GeckoModule::closePorts()
{
    //-- Input port
    input_port.disableCallback();
    input_port.interrupt();
    input_port.close();

    //-- Gesture port
    gesture_port.interrupt();
    gesture_port.close();

    //-- Position port
    position_port.interrupt();
    position_port.close();

    //-- Debug port
    if (debugOn)
    {
        debug_port.interrupt();
        debug_port.close();
    }

    //-- Segmentation Debug port
    if (segmentationDebugOn)
    {
        segmentation_debug_port.interrupt();
        segmentation_debug_port.close();
    }
}
