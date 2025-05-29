cmake_minimum_required(VERSION 3.22)


add_subdirectory(libtelebot)


include_directories(libtelebot/include)
add_library(ufr_gtw_telegram SHARED ./src/ufr_gtw_telegram.c)
link_libraries(ufr_gtw_telegram libtelebot)