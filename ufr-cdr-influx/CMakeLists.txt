cmake_minimum_required(VERSION 3.5)
project(ufr_cdr_influx)

add_library(ufr_cdr_influx SHARED ./src/ufr_enc_influx.c) # ./src/ufr_dcr_influx.c)
target_link_libraries(ufr_cdr_influx ${MSGPACK_LIBS} ufr)
target_compile_options(ufr_cdr_influx PUBLIC -O1)
install(TARGETS ufr_cdr_influx DESTINATION lib)

# test
if(BUILD_TESTS)
    enable_testing()
    # add_subdirectory(./tests)
endif(BUILD_TESTS)