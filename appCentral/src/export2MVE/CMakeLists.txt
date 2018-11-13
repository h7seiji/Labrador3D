add_executable(export2MVE export2MVE.cpp)
target_include_directories(export2MVE PUBLIC ${OPENMVG_INCLUDE_DIRS})
target_link_libraries(export2MVE ${OPENMVG_LIBRARIES})
