#include "GeckoModule.hpp"


const float gecko::GeckoModule::MODULE_PERIOD = 3.0;


const std::string gecko::GeckoModule::PORT_PREFIX = "/gecko";
const std::string gecko::GeckoModule::INPUT_PORT = "/src:i";
const std::string gecko::GeckoModule::DEBUG_PORT = "/debug:o";
const std::string gecko::GeckoModule::GESTURE_PORT = "/gesture:o";
const std::string gecko::GeckoModule::POSITION_PORT = "/handPos:o";


gecko::GeckoModule::GeckoModule()
{

}

bool gecko::GeckoModule::configure(yarp::os::ResourceFinder &rf)
{
    return false;
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

}
