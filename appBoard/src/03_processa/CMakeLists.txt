add_executable(03_process processa.cpp)
target_include_directories(03_process PUBLIC ${MVG_INCLUDES})
target_link_libraries(03_process 
  PRIVATE
    OpenMVG::openMVG_numeric
    OpenMVG::openMVG_image
    OpenMVG::openMVG_features
)

# version 1.3
#${OPENMVG_INCLUDE_DIRS}
# ${OPENMVG_LIBRARIES}

#version 1.4
#${MVG_INCLUDES}
#PRIVATE
#    OpenMVG::openMVG_numeric
#    OpenMVG::openMVG_image
#    OpenMVG::openMVG_features
