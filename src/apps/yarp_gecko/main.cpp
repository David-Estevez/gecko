// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

//------------------------------------------------------------------------------
//-- Gecko_image_analyzer
//------------------------------------------------------------------------------
//--
//-- Demonstration of the Gecko main program features by analyzing static images
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

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

//#include "ColorDebug.hpp"
#include "GeckoModule.hpp"

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cylinderTracker");
    //rf.setDefaultConfigFile("cylinderTracker.ini");
    rf.configure(argc, argv);

//    CD_INFO("Checking for yarp network...\n");
    yarp::os::Network::init();
//    if (!yarp.checkNetwork()) {
//        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
//        return -1;
//    }
//    CD_SUCCESS("Found yarp network.\n");

    GeckoModule gecko;
    return gecko.runModule(rf);
}
