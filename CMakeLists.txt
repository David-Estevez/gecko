#MAIN 

cmake_minimum_required(VERSION 2.8)

project(GECKO)

FIND_PACKAGE( OpenCV REQUIRED )


set (RBCP_TRUNK_DIR "${PROJECT_SOURCE_DIR}")


set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")

add_subdirectory (src)


