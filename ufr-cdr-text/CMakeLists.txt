cmake_minimum_required(VERSION 3.5)
project(ufr-cdr-text)

# libcdr_csv.so
add_library(ufr_cdr_text SHARED ./src/ufr_enc_text.c ./src/ufr_dcr_text.c)
target_compile_options(ufr_cdr_text PUBLIC -O1)
target_link_libraries(ufr_cdr_text ufr)
install(TARGETS ufr_cdr_text DESTINATION lib)

# test
if(BUILD_TESTS)
   enable_testing()
   add_subdirectory(./tests)
endif(BUILD_TESTS)