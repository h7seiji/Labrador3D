cmake_minimum_required(VERSION 2.8)
project( labrador3D )

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(OpenMVG REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(01_recebe_comandos)
add_subdirectory(02_tira_foto)
add_subdirectory(03_processa)
add_subdirectory(04_envia_imagem)
