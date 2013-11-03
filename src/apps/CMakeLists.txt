# src/apps

add_executable( hand hand.cpp)
TARGET_LINK_LIBRARIES( hand HandUtils HandDetector FingerDetector Mouse HandDescription ${OpenCV_LIBS} )

add_executable( gesture_recognition gesture_recognition.cpp)
TARGET_LINK_LIBRARIES( gesture_recognition HandUtils HandDetector FingerDetector Mouse HandDescription ${OpenCV_LIBS} )

add_executable( finger_detector_pictures finger_detector_pictures.cpp)
TARGET_LINK_LIBRARIES( finger_detector_pictures HandUtils HandDetector FingerDetector ${OpenCV_LIBS} )

add_executable(fingers fingers.cpp)
TARGET_LINK_LIBRARIES( fingers ${OpenCV_LIBS} )
