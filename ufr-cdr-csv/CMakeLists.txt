cmake_minimum_required(VERSION 3.5)
project(ufr_cdr_csv)

# libufr_cdr_csv.so
add_library(ufr_cdr_csv SHARED ./src/ufr_enc_csv.cpp ./src/ufr_dcr_csv.c)
target_compile_options(ufr_cdr_csv PUBLIC -O1)
target_link_libraries(ufr_cdr_csv ufr)
install(TARGETS ufr_cdr_csv DESTINATION lib)

# test
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(./tests)
endif(BUILD_TESTS)