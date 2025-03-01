add_executable(exemplo_publisher publisher.c)
target_link_libraries(exemplo_publisher ufr)
install(TARGETS exemplo_publisher DESTINATION examples)

find_package(OpenCV)

add_executable(teste teste.cpp)
target_link_libraries(teste  ufr ${OpenCV_LIBS})
target_include_directories(teste PUBLIC "${OPENCV_INCLUDE_DIR}")

install(TARGETS teste DESTINATION examples)
