
IF(ENABLE_YARP_module)

# Find YARP.  Point the YARP_DIR environment variable at your build.
find_package(YARP REQUIRED)
find_package( OpenCV REQUIRED )
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

# Include any directories needed
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${GECKO_INCLUDE_DIRS})
include_directories(${COLOR_DEBUG_INCLUDE_DIRS})

# Add executable:
add_executable(yarp_gecko main.cpp GeckoModule.cpp)
target_link_libraries(yarp_gecko ${GECKO_LIBRARIES} ${YARP_LIBRARIES} ${OpenCV_LIBS} )

ENDIF(ENABLE_YARP_module)
