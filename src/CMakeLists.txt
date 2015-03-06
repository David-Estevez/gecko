# src

set(GECKO_INCLUDE_DIRS ${GECKO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/libraries CACHE INTERNAL "appended header dirs")

add_subdirectory(libraries)
add_subdirectory(apps)


