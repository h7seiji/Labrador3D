add_executable(03_process processa.cpp)

if( ${OPENMVG_VERSION} MATCHES "1.3.0" )
	message("OpenMVG version 1.3.0 FOUND!")
	target_include_directories(03_process PUBLIC ${OPENMVG_INCLUDE_DIRS})
	target_link_libraries(03_process ${OPENMVG_LIBRARIES})
else()
	message("OpenMVG version: ${OPENMVG_VERSION}")
	set(MVG_INCLUDES /usr/local/include/openMVG_dependencies/cereal/include)
	target_include_directories(03_process PUBLIC ${MVG_INCLUDES})
	target_link_libraries(03_process 
	  PRIVATE
		OpenMVG::openMVG_numeric
		OpenMVG::openMVG_image
		OpenMVG::openMVG_features
	)
endif()

# version 1.3
#${OPENMVG_INCLUDE_DIRS}
# ${OPENMVG_LIBRARIES}

#version 1.4
#${MVG_INCLUDES}
#PRIVATE
#    OpenMVG::openMVG_numeric
#    OpenMVG::openMVG_image
#    OpenMVG::openMVG_features
