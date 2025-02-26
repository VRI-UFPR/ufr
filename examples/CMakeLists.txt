cmake_minimum_required(VERSION 3.5)
project(teste)

add_executable(teste teste.c)
target_link_libraries(teste ufr)
install(TARGETS teste DESTINATION examples)
