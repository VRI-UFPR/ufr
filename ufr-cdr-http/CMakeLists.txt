cmake_minimum_required(VERSION 3.5)
project(cdr_http)

add_library(ufr_cdr_http SHARED 
    ./src/ufr_enc_http_req.c 
    ./src/ufr_enc_http_ans.c 
    ./src/ufr_dcr_http_req.c 
    ./src/ufr_dcr_http_ans.c 
    ./src/picohttpparser.c)

target_link_libraries(ufr_cdr_http ufr)
target_compile_options(ufr_cdr_http PUBLIC -O1)
install(TARGETS ufr_cdr_http DESTINATION lib)

# test
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(./tests)
endif(BUILD_TESTS)