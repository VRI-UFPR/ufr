cmake_minimum_required(VERSION 3.5)
project(ufr-gtw-chart)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK4 REQUIRED IMPORTED_TARGET gtk4)

add_library(ufr_gtw_chart SHARED ./src/ufr_gtw_chart.c)
target_compile_options(ufr_gtw_chart PUBLIC -O1)
target_link_libraries(ufr_gtw_chart ufr PkgConfig::GTK4)
install(TARGETS ufr_gtw_chart DESTINATION lib)

add_library(ufr_gtw_chart2d SHARED ./src/ufr_gtw_chart2d.c)
target_compile_options(ufr_gtw_chart2d PUBLIC -O1)
target_link_libraries(ufr_gtw_chart2d ufr PkgConfig::GTK4)
install(TARGETS ufr_gtw_chart2d DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)