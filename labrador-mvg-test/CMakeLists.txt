# SUGESTED COMMANDS
# cmake . -Bbuild
# make -Cbuild
# ./main

cmake_minimum_required(VERSION 2.8)
project( display )

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR})

find_package(OpenMVG REQUIRED)

set(MVG_INCLUDES /usr/local/include/openMVG_dependencies/cereal/include)
include_directories(${MVG_INCLUDES})

add_executable(main labrador-mvg-test-2.cpp)

target_link_libraries(main
  PRIVATE
    OpenMVG::openMVG_numeric
    OpenMVG::openMVG_image
    OpenMVG::openMVG_features
)


