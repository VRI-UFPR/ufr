cmake_minimum_required(VERSION 3.22)
project(ufr_gtw_gnuplot)

add_library(ufr_gtw_gnuplot SHARED 
    ./src/ufr_gtw_gnuplot.c
    )
# target_link_libraries(ufr_gtw_telegram ${curl_LDFLAGS} ${json_LDFLAGS} ufr)
# SET_TARGET_PROPERTIES(ufr_gtw_telegram PROPERTIES VERSION ${VERSION})


# add_executable(example_telegram ./examples/bot.c)
# target_link_libraries(example_telegram ufr ufr_gtw_telegram)
