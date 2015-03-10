// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

//------------------------------------------------------------------------------
//-- GeckoModule
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

#ifndef __GECKO_MODULE__
#define __GECKO_MODULE__

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>

#include <string>

#include "ColorDebug.hpp"

#include "HandDetector.h"
#include "HandDescriptor.h"

namespace gecko
{

typedef yarp::sig::ImageOf<yarp::sig::PixelRgb> Image;

/**
 * @ingroup GeckoModule
 *
 * A YARP module to track cylinder from point cloud
 *
 */
class GeckoModule : public yarp::os::RFModule, public yarp::os::TypedReaderCallback<Image>
{
    public:
        GeckoModule();
        bool configure(yarp::os::ResourceFinder& rf);

        static const float MODULE_PERIOD;

        //-- Port name definitions
        static const std::string PORT_PREFIX;
        static const std::string INPUT_PORT;
        static const std::string DEBUG_PORT;
        static const std::string SEGMENTATION_DEGUB_PORT;
        static const std::string GESTURE_PORT;
        static const std::string POSITION_PORT;

        void onRead(Image& src);

    protected:
        virtual double getPeriod();
        virtual bool updateModule();
        virtual bool close();

    private:
        //-- Ports
        yarp::os::BufferedPort<Image> input_port;
        yarp::os::BufferedPort<Image> debug_port;
        yarp::os::BufferedPort<Image> segmentation_debug_port;
        yarp::os::BufferedPort<yarp::os::Bottle> gesture_port;
        yarp::os::BufferedPort<yarp::os::Bottle> position_port;

        bool openPorts();
        bool closePorts();

        bool debugOn;
        bool segmentationDebugOn;

        bool connectInput;
        std::string rgbStreamPort;

        HandDetector handDetector;
        HandDescriptor handDescriptor;

};

}  // namespace gecko

#endif  // __GECKO_MODULE__
