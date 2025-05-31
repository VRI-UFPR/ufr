cmake_minimum_required(VERSION 3.5)
project(ufr-gtw-file)

# sudo apt-get install libcurl4-openssl-dev
add_library(ufr_gtw_file SHARED
    ./src/ufr_gtw_file.c
    )

target_compile_options(ufr_gtw_file PUBLIC -O3)
target_link_libraries(ufr_gtw_file ufr curl)
install(TARGETS ufr_gtw_file DESTINATION lib)

# test
# if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(./tests)
# endif(BUILD_TESTS)