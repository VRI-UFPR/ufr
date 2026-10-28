cmake_minimum_required(VERSION 3.5)
project(cdr_json)


add_library(ufr_cdr_json SHARED 
    ./src/ufr_enc_json.c 
    )
target_link_libraries(ufr_cdr_json ufr)
target_compile_options(ufr_cdr_json PUBLIC -O3)
install(TARGETS ufr_cdr_json DESTINATION lib)

# test
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(./tests)
endif(BUILD_TESTS)