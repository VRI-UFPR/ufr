cmake_minimum_required(VERSION 3.5)
project(cc_msgpack)

# find_package(msgpackc REQUIRED)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(msgpackc IMPORTED_TARGET libmsgpackc REQUIRED)

# REMEMBER
# MSGPACK_LIBS = msgpack-c (ubuntu22) or msgpack-c (ubuntu24)


add_library(ufr_enc_msgpack SHARED ./src/ufr_enc_msgpack.c)
target_link_libraries(ufr_enc_msgpack ${MSGPACK_LIBS} ufr)
target_compile_options(ufr_enc_msgpack PUBLIC -O1)
install(TARGETS ufr_enc_msgpack DESTINATION lib)

add_library(ufr_dcr_msgpack SHARED ./src/ufr_dcr_msgpack.c)
target_link_libraries(ufr_dcr_msgpack ${MSGPACK_LIBS} ufr)
target_compile_options(ufr_dcr_msgpack PUBLIC -O1)
install(TARGETS ufr_dcr_msgpack DESTINATION lib)

# test
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(./tests)
endif(BUILD_TESTS)