add_executable(takePictures takePictures.cpp)
#target_include_directories(takePictures PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(takePictures ${CMAKE_THREAD_LIBS_INIT}) #${OpenCV_LIBS})
