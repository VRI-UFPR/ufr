cmake_minimum_required(VERSION 3.5)
project(ufr-stk-ros_melodic)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(./src)

# Gateway
add_library(ufr_gtw_ros_melodic SHARED src/ufr_gtw_ros_melodic.cpp)
target_link_libraries(ufr_gtw_ros_melodic ufr ${catkin_LIBRARIES})
install(TARGETS ufr_gtw_ros_melodic DESTINATION lib)

# Encoder
add_library(ufr_enc_ros_melodic SHARED 
    src/sensor_msgs/ufr_enc_image.cpp
    src/geometry_msgs/ufr_enc_pose.cpp
    src/sensor_msgs/ufr_enc_laserscan.cpp
    )
target_link_libraries(ufr_enc_ros_melodic ufr ${catkin_LIBRARIES})
install(TARGETS ufr_enc_ros_melodic DESTINATION lib)

# Decoder
add_library(ufr_dcr_ros_melodic SHARED 
    src/std_msgs/ufr_dcr_i16.cpp
    src/sensor_msgs/ufr_dcr_laserscan.cpp
    src/sensor_msgs/ufr_dcr_range.cpp
    src/sensor_msgs/ufr_dcr_imu.cpp
    src/sensor_msgs/ufr_dcr_navsatfix.cpp
    )
target_link_libraries(ufr_dcr_ros_melodic ufr ${catkin_LIBRARIES})
install(TARGETS ufr_dcr_ros_melodic DESTINATION lib)


# Tests
# enable_testing()
# add_subdirectory(./tests)
