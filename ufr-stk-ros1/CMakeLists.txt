cmake_minimum_required(VERSION 3.5)
project(ufr-stk-ros1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(./src/)

## Declare ROS messages and services
# add_message_files(DIRECTORY msg FILES Num.msg)
# add_service_files(DIRECTORY srv FILES AddTwoInts.srv)
# generate_messages(DEPENDENCIES std_msgs)
catkin_package()


add_library(ufr_gtw_ros1 SHARED src/ufr_gtw_ros1.cpp)
target_link_libraries(ufr_gtw_ros1 ${catkin_LIBRARIES})
install(TARGETS ufr_gtw_ros1 DESTINATION lib)

add_library(ufr_enc_ros1 SHARED
    # src/std_msgs/ufr_enc_string.cpp
    src/std_msgs/ufr_enc_tf.cpp
    # src/geometry_msgs/ufr_enc_twist.cpp
    # src/sensor_msgs/ufr_enc_laserscan.cpp 
    # src/sensor_msgs/ufr_enc_image.cpp
    )
target_link_libraries(ufr_enc_ros_noetic ${catkin_LIBRARIES})

add_library(ufr_dcr_ros1 SHARED
    # src/std_msgs/ufr_dcr_string.cpp
    src/sensor_msgs/ufr_dcr_laserscan.cpp 
    src/sensor_msgs/ufr_dcr_range.cpp 
    src/sensor_msgs/ufr_dcr_navsatfix.cpp
    src/sensor_msgs/ufr_dcr_image.cpp
    src/sensor_msgs/ufr_dcr_imu.cpp
    src/std_msgs/ufr_dcr_i16.cpp
    )
target_link_libraries(ufr_dcr_ros1 ${catkin_LIBRARIES})
install(TARGETS ufr_dcr_ros1 DESTINATION lib)

# add_library(ufr_enc_ros_noetic SHARED src/ufr_enc_string.cpp)
# target_link_libraries(ufr_enc_ros_noetic ${catkin_LIBRARIES})

# Tests
# enable_testing()
# add_subdirectory(./tests)