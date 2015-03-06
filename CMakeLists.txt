#MAIN 

cmake_minimum_required(VERSION 2.8)

project(GECKO)

FIND_PACKAGE( OpenCV REQUIRED )

# Options
option(ENABLE_YARP_module "Choose if you want to compile the yarp module version of GECKO" FALSE)


# Dirs where the ouptut files will go
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
message(STATUS "Libraries go to ${LIBRARY_OUTPUT_PATH}")
message(STATUS "Executables go to ${EXECUTABLE_OUTPUT_PATH}")

# this doesn't happen automatically for makefiles
make_directory(${LIBRARY_OUTPUT_PATH})
make_directory(${EXECUTABLE_OUTPUT_PATH})

# To get gecko libraries include path
set(GECKO_INCLUDE_DIRS CACHE INTERNAL "appended header dirs" FORCE)
set(GECKO_LINK_DIRS CACHE INTERNAL "appended link dirs" FORCE)
set(GECKO_LIBRARIES CACHE INTERNAL "appended libraries" FORCE)

add_subdirectory(src)


