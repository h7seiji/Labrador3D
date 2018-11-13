add_executable(dataInit dataInit.cpp)
target_include_directories(dataInit PUBLIC ${OPENMVG_INCLUDE_DIRS})
target_link_libraries(dataInit ${OPENMVG_LIBRARIES})
