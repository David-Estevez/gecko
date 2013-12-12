# src/apps

add_executable( hand hand.cpp)
TARGET_LINK_LIBRARIES( hand HandUtils HandDetector Mouse HandDescription StateMachine AppLauncher ${OpenCV_LIBS} )

add_executable( gecko_image_analyzer image_analyzer.cpp)
target_link_libraries( gecko_image_analyzer HandUtils HandDetector HandDescription ${OpenCV_LIBS} )
