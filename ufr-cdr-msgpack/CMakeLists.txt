cmake_minimum_required(VERSION 3.5)
project(cdr_msgpack)

# find_package(msgpackc)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(msgpackc IMPORTED_TARGET libmsgpackc REQUIRED)

# REMEMBER
# MSGPACK_LIBS = msgpack-c (ubuntu22) or msgpack-c (ubuntu24)


add_library(ufr_cdr_msgpack SHARED ./src/ufr_enc_msgpack.c ./src/ufr_dcr_msgpack.c ./src/ufr_enc_msgpack_dict.c ./src/ufr_dcr_msgpack_array.c)
# target_link_libraries(ufr_cdr_msgpack ${MSGPACK_LIBS} ufr)
target_link_libraries(ufr_cdr_msgpack msgpackc ufr)
target_compile_options(ufr_cdr_msgpack PUBLIC -O3)
install(TARGETS ufr_cdr_msgpack DESTINATION lib)

# test
# if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(./tests)
# endif(BUILD_TESTS)