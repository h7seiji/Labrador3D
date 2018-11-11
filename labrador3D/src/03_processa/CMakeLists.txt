add_executable(03_process processa.cpp)
target_include_directories(03_process PUBLIC ${OPENMVG_INCLUDE_DIRS})
target_link_libraries(03_process ${OPENMVG_LIBRARIES})
