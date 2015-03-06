# src/libraries

include_directories(${GECKO_INCLUDE_DIRS})


ADD_LIBRARY( HandDetector HandDetector.cpp)
TARGET_LINK_LIBRARIES (HandDetector HandUtils)

ADD_LIBRARY( HandDescriptor HandDescriptor.cpp)
TARGET_LINK_LIBRARIES (HandDescriptor HandUtils Mouse)

ADD_LIBRARY( HandUtils handUtils.cpp)

ADD_LIBRARY( Mouse mouse.cpp)
TARGET_LINK_LIBRARIES (Mouse X11)

ADD_LIBRARY( AppLauncher AppLauncher.cpp )
TARGET_LINK_LIBRARIES (AppLauncher StateMachine)

ADD_LIBRARY( StateMachine StateMachine.cpp )


# Export include path
set(GECKO_LIBRARIES ${GECKO_LIBRARIES} HandDetector HandDescriptor HandUtils Mouse AppLauncher StateMachine  CACHE INTERNAL "appended libraries")


