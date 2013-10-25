# src/libraries
ADD_LIBRARY( HandDetector HandDetector.cpp)
ADD_LIBRARY( HandDescription HandDescription.cpp)
ADD_LIBRARY( HandUtils handUtils.cpp)
ADD_LIBRARY( FingerDetector FingerDetector.cpp)
ADD_LIBRARY( Mouse mouse.cpp)

TARGET_LINK_LIBRARIES (Mouse X11)
