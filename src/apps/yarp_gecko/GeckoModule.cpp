#include "GeckoModule.hpp"


const float gecko::GeckoModule::MODULE_PERIOD = 3.0;


const std::string gecko::GeckoModule::PORT_PREFIX = "/gecko";
const std::string gecko::GeckoModule::INPUT_PORT = "/src:i";
const std::string gecko::GeckoModule::DEBUG_PORT = "/debug:o";
const std::string gecko::GeckoModule::GESTURE_PORT = "/gesture:o";
const std::string gecko::GeckoModule::POSITION_PORT = "/handPos:o";


gecko::GeckoModule::GeckoModule()
{
    debugOn = false;
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
            CD_ERROR("Could not open debug output port at %s\n", (PORT_PREFIX+POSITION_PORT).c_str());
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
}
