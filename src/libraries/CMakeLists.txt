# src/libraries
ADD_LIBRARY( HandDetector HandDetector.cpp)
ADD_LIBRARY( HandDescription HandDescription.cpp)
ADD_LIBRARY( HandUtils handUtils.cpp)
ADD_LIBRARY( Mouse mouse.cpp)
ADD_LIBRARY( AppLauncher AppLauncher.cpp )
ADD_LIBRARY( StateMachine StateMachine.cpp )

TARGET_LINK_LIBRARIES (Mouse X11)
TARGET_LINK_LIBRARIES (HandDescription HandUtils Mouse)
TARGET_LINK_LIBRARIES (HandDetector HandUtils)
TARGET_LINK_LIBRARIES (AppLauncher StateMachine)

