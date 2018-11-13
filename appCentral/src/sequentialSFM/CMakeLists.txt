add_executable(sequentialSFM sequentialSFM.cpp)
target_include_directories(sequentialSFM PUBLIC ${OPENMVG_INCLUDE_DIRS})
target_link_libraries(sequentialSFM ${OPENMVG_LIBRARIES})
