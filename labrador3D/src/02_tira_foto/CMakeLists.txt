add_executable(02_picture placa_tirar_foto.cpp)
target_include_directories(02_picture PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(02_picture ${OpenCV_LIBS})
