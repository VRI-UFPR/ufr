cmake_minimum_required(VERSION 3.5)
project(ufr)

# Check msgpack library <= Ubuntu 23
set(HAVE_MSGPACK OFF)
find_package(msgpack)
if (msgpack_DIR)
    set(HAVE_MSGPACK ON)
    set(MSGPACK_LIBS msgpackc)
else()
    # Check msgpack library >= Ubuntu 24
    find_package(msgpack-c)
    if (msgpack-c_DIR)
        set(HAVE_MSGPACK ON)
        set(MSGPACK_LIBS msgpack-c)
    endif()
endif()

# Check mosquitto library
find_package(PkgConfig)
set(HAVE_MOSQUITTO OFF)
pkg_check_modules(Mosquitto IMPORTED_TARGET libmosquitto)
if (Mosquitto_FOUND)
    set(HAVE_MOSQUITTO ON)
endif()

# Check zmq library
set(HAVE_ZMQ OFF)
pkg_check_modules(ZMQ IMPORTED_TARGET libzmq)
if (ZMQ_FOUND)
    set(HAVE_ZMQ ON)
endif()

add_compile_options(-Wall)
# add_compile_options(-DCMAKE_BUILD_TYPE=Release)

set(HAVE_OPENCV OFF)
find_package(OpenCV)
if (OPENCV_FOUND)
    set(HAVE_OPENCV ON)
endif()

# Configure compilation
option(BUILD_UFR_GTW_ZMQ      "build the gateway for ZMQ"      ${HAVE_ZMQ})
option(BUILD_UFR_GTW_MQTT     "build the gateway for MQTT"     ${HAVE_MOSQUITTO})
option(BUILD_UFR_CDR_MSGPACK  "build the coder for MsgPack"    ${HAVE_MSGPACK})
option(BUILD_UFR_STK_SQLITE   "build the stack for Sqlite3"    OFF )
option(BUILD_UFR_STK_ROS1     "build the stack for ROS1"       OFF )
option(BUILD_UFR_STK_ROS2     "build the stack for ROS2"       OFF )
option(BUILD_UFR_DEV_VIDEO    "build the stack for Video"      ${HAVE_OPENCV} )
option(BUILD_UFR_STK_TELEGRAM "build the server for Telegram"  OFF )
option(BUILD_EXAMPLES         "build the examples"             OFF )
option(BUILD_TESTS            "build the tests"                ON )
option(COVERAGE_TEST          "coverage test"                  OFF)
# option(BUILD_APPLICATION        "build the applications"       ON )

if(COVERAGE_TEST)
    SET(CMAKE_C_FLAGS "-fprofile-arcs -ftest-coverage --coverage -g -O0")
    SET(GCC_COVERAGE_LINK_FLAGS "-lgcov")
    # gcovr -r ..
endif(COVERAGE_TEST)

# include UFR API
include_directories(./ufr-api-c/include)

# Build libraries with no dependencies
add_subdirectory(./ufr-api-c)
add_subdirectory(./ufr-gtw-posix)
add_subdirectory(./ufr-cdr-csv)
add_subdirectory(./ufr-cdr-text)

# requer libcurl-...-dev
# add_subdirectory(./ufr-gtw-file)

# Build libraries with dependencies
if(BUILD_UFR_GTW_ZMQ)
    # debian/ubuntu: sudo apt install libzmq3-dev
    add_subdirectory(./ufr-gtw-zmq)
endif(BUILD_UFR_GTW_ZMQ)

if(BUILD_UFR_GTW_MQTT)
    # debian/ubuntu: sudo apt install libmosquitto-dev
    add_subdirectory(./ufr-gtw-mqtt)
endif(BUILD_UFR_GTW_MQTT)

if(BUILD_UFR_CDR_MSGPACK)
    # debian/ubuntu: sudo apt install libmsgpack-dev
    add_subdirectory(./ufr-cdr-msgpack)
endif(BUILD_UFR_CDR_MSGPACK)

if(BUILD_UFR_STK_SQLITE)
    # debian/ubuntu: sudo apt install libsqlite3-dev
    add_subdirectory(./ufr-stk-sqlite)
endif(BUILD_UFR_STK_SQLITE)

# Build examples
if(BUILD_EXAMPLES)
    add_subdirectory(./examples/)
    install(FILES ./etc/init.bash DESTINATION .)
endif(BUILD_EXAMPLES)

# Build tests
if(BUILD_TESTS)
    # add_subdirectory(./tests/)
    enable_testing()
    # add_test(NAME test_gtw_zmq COMMAND ./tests/test_topic "@new zmq:topic @port 2000")
endif(BUILD_TESTS)

if(BUILD_UFR_DEV_VIDEO)
    # debian/ubuntu: sudo apt install libopencv-dev
    add_subdirectory(./ufr-dev-video)
    # include_directories(./cv_mat/include)
    # add_subdirectory(./cv_mat)
endif(BUILD_UFR_DEV_VIDEO)

# add_subdirectory(./ufr-app-filesystem)
# add_subdirectory(./ufr-app-proxy)
# add_subdirectory(./ufr-gtw-qt)
# add_subdirectory(./ufr-gtw-gtk)
# add_subdirectory(./ufr-cdr-http)
# add_subdirectory(./ufr-gtw-http)

# debian/ubuntu: sudo apt install libssh-dev
# add_subdirectory(./ufr-stk-ssh)
# add_subdirectory(ufr-stk-irrlicht)
# add_subdirectory(./ufr-stk-webots)
# add_subdirectory(./ufr-gtw-chart)
# add_subdirectory(./cv_mat)

if(BUILD_UFR_STK_ROS1)
    add_subdirectory(./ufr-stk-ros1)
endif(BUILD_UFR_STK_ROS1)

if(BUILD_UFR_STK_ROS2)
    add_subdirectory(./ufr-stk-ros2)
endif(BUILD_UFR_STK_ROS2)



# install(FILES ./etc/Makefile DESTINATION .)

