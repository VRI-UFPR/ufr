cmake_minimum_required(VERSION 3.5)
project(ufr_cdr_cbor)

# sudo apt install libcbor-dev

# libufr_cdr_cbor.so
add_library(ufr_cdr_cbor SHARED ./src/ufr_enc_cbor.c)
target_link_libraries(ufr_cdr_cbor ufr cbor)
target_compile_options(ufr_cdr_cbor PUBLIC -O1)
install(TARGETS ufr_cdr_cbor DESTINATION lib)

# test
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(./tests)
endif(BUILD_TESTS)