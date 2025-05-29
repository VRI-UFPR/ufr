cmake_minimum_required(VERSION 3.5)
project(ufr-cdr-text)

# libenc_csv.so
add_library(ufr_enc_text SHARED ./src/ufr_enc_text.c)
target_compile_options(ufr_enc_text PUBLIC -O1)
target_link_libraries(ufr_enc_text ufr)
install(TARGETS ufr_enc_text DESTINATION lib)

# libdcr_csv.so
add_library(ufr_dcr_text SHARED ./src/ufr_dcr_text.c)
target_compile_options(ufr_dcr_text PUBLIC -O1)
target_link_libraries(ufr_dcr_text ufr)
install(TARGETS ufr_dcr_text DESTINATION lib)

# test
if(BUILD_TESTS)
   enable_testing()
   add_subdirectory(./tests)
endif(BUILD_TESTS)