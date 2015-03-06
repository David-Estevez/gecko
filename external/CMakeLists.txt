cmake_minimum_required(VERSION 2.8)

include(ExternalProject)

ExternalProject_Add(ColorDebug
    DOWNLOAD_COMMAND ""
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/external/ColorDebug
    INSTALL_COMMAND ""
)

set(GECKO_INCLUDE_DIRS ${GECKO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/ColorDebug CACHE INTERNAL "appended header dirs")
