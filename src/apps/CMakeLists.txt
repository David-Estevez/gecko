# src/apps

add_executable(fingers fingers.cpp)
TARGET_LINK_LIBRARIES( fingers ${OpenCV_LIBS} )

add_executable( hand hand.cpp)
TARGET_LINK_LIBRARIES( hand ${OpenCV_LIBS} )
