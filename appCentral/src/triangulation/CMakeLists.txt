add_executable(triangulation triangulation.cpp)
target_include_directories(triangulation PUBLIC ${OPENMVG_INCLUDE_DIRS})
target_link_libraries(triangulation ${OPENMVG_LIBRARIES})
