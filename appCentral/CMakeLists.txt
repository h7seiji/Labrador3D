cmake_minimum_required(VERSION 2.8)
project( hey3D )

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(OpenMVG REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(Threads)

add_subdirectory(src/dataInit)
add_subdirectory(src/takePictures)
add_subdirectory(src/sequentialSFM)
add_subdirectory(src/triangulation)
add_subdirectory(src/export2MVE)
