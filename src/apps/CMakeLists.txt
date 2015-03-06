# src/apps

# Inclue any directories needed
include_directories(${GECKO_INCLUDE_DIRS})

add_executable( gecko gecko.cpp)
TARGET_LINK_LIBRARIES( gecko HandUtils HandDetector Mouse HandDescriptor StateMachine AppLauncher ${OpenCV_LIBS} )

add_executable( gecko_image_analyzer image_analyzer.cpp)
target_link_libraries( gecko_image_analyzer HandUtils HandDetector HandDescriptor ${OpenCV_LIBS} )

add_subdirectory(yarp_gecko)
