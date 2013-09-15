# src/apps

add_executable(fingers fingers.cpp)
TARGET_LINK_LIBRARIES( fingers ${OpenCV_LIBS} )
