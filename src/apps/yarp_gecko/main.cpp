// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

//------------------------------------------------------------------------------
//-- yarp_gecko
//------------------------------------------------------------------------------
//--
//-- YARP module to track hands and gestures
//--
//------------------------------------------------------------------------------
//--
//-- This file belongs to the "Gecko - Gesture Recognition" project
//-- (https://github.com/David-Estevez/gecko)
//--
//------------------------------------------------------------------------------
//-- Authors: David Estevez Fernandez
//--
//-- Released under the GPL license (more info on LICENSE.txt file)
//------------------------------------------------------------------------------

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include "ColorDebug.hpp"
#include "GeckoModule.hpp"

using namespace gecko;

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("yarp_module");
    //rf.setDefaultConfigFile("yarp_module.ini");
    rf.configure(argc, argv);

    CD_INFO("Checking for yarp network...\n");
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork()) {
        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return -1;
    }
    CD_SUCCESS("Found yarp network.\n");

    GeckoModule gecko;
    return gecko.runModule(rf);
}
