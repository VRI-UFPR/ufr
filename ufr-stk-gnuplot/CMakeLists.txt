cmake_minimum_required(VERSION 3.22)
project(ufr_gtw_gnuplot)

add_library(ufr_gtw_gnuplot SHARED 
    ./src/ufr_gtw_gnuplot.c
    )
# target_link_libraries(ufr_gtw_gnuplot ufr)
# SET_TARGET_PROPERTIES(ufr_gtw_gnuplot PROPERTIES VERSION ${VERSION})


add_executable(example_gnuplot ./examples/chart1.c)
target_link_libraries(example_gnuplot ufr ufr_gtw_gnuplot m)
