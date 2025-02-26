cmake_minimum_required(VERSION 3.5)
project(teste)

find_package(OpenCV)

add_executable(teste teste.cpp)
target_link_libraries(teste  ufr ${OpenCV_LIBS})
target_include_directories(teste PUBLIC "${OPENCV_INCLUDE_DIR}")

install(TARGETS teste DESTINATION examples)
