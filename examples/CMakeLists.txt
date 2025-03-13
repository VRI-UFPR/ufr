find_package(OpenCV)

add_executable(teste teste.cpp)
target_link_libraries(teste  ufr ${OpenCV_LIBS})
target_include_directories(teste PUBLIC "${OPENCV_INCLUDE_DIR}")

# add_executable(teste2 teste_noetic_magrin.c)
# target_link_libraries(teste2  ufr)

install(TARGETS teste DESTINATION examples)
