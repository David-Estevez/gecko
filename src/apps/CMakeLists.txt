# src/apps

add_executable(fingers fingers.cpp)
TARGET_LINK_LIBRARIES( fingers ${OpenCV_LIBS} )

add_executable( hand hand.cpp)
TARGET_LINK_LIBRARIES( hand HandUtils HandDetector FingerDetector Mouse ${OpenCV_LIBS} )

add_executable( finger_detector_pictures finger_detector_pictures.cpp)
TARGET_LINK_LIBRARIES( finger_detector_pictures HandUtils HandDetector FingerDetector ${OpenCV_LIBS} )
