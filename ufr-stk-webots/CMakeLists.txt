cmake_minimum_required(VERSION 3.5)
project(ufr-stk-webots)

link_directories(/usr/local/webots/lib/controller/)
include_directories(/usr/local/webots/include/controller/c/)
include_directories(./src)

add_library(ufr_gtw_webots SHARED
    ./src/gtw_webots.c
    ./src/enc_motors.c
    ./src/enc_motors_drone.c
    ./src/dcr_encoders.c
    ./src/dcr_lidar.c
    ./src/dcr_pose.c
    )

target_compile_options(ufr_gtw_webots PUBLIC -O1)
target_link_libraries(ufr_gtw_webots ufr Controller)
install(TARGETS ufr_gtw_webots DESTINATION lib)

# test
enable_testing()
add_subdirectory(./tests)

# Examples
add_executable(ex01_cmdvel ./examples/ex01_cmdvel.c)
target_link_libraries(ex01_cmdvel ufr)
add_executable(ex02_pose ./examples/ex02_pose.c)
target_link_libraries(ex02_pose ufr)
add_executable(ex03_lidar ./examples/ex03_lidar.c)
target_link_libraries(ex03_lidar ufr)