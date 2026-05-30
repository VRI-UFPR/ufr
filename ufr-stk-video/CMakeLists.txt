cmake_minimum_required(VERSION 3.5)
project(ufr_dev_video)
find_package(OpenCV)

# Library
add_library(ufr_gtw_video SHARED
    ./src/ufr_stk_video_opencv.cpp
    ./src/ufr_stk_video_link.cpp
    ./src/ufr_gtw_video.c
    )
target_compile_options(ufr_gtw_video PUBLIC -O1)
target_link_libraries(ufr_gtw_video  ufr ${OpenCV_LIBS})
target_include_directories(ufr_gtw_video PUBLIC "${OPENCV_INCLUDE_DIR}")
install(TARGETS ufr_gtw_video DESTINATION lib)

# Example
# if (BUILD_EXAMPLES)
    add_executable(ufr_dev_video_sub ./examples/subscriber.cpp)
    target_compile_options(ufr_dev_video_sub PUBLIC -O1)
    target_link_libraries(ufr_dev_video_sub  ufr ${OpenCV_LIBS})
    target_include_directories(ufr_dev_video_sub PUBLIC "${OPENCV_INCLUDE_DIR}")

    add_executable(ufr_dev_video_pub ./examples/publisher.cpp)
    target_compile_options(ufr_dev_video_pub PUBLIC -O1)
    target_link_libraries(ufr_dev_video_pub  ufr ${OpenCV_LIBS})
    target_include_directories(ufr_dev_video_pub PUBLIC "${OPENCV_INCLUDE_DIR}")
# endif(BUILD_EXAMPLES)

# test
# enable_testing()
# add_subdirectory(./tests)
