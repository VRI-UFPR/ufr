include_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(test1 ./test1.cpp)
target_link_libraries(test1 ufr ufr_gtw_ros2 ufr_enc_ros2 ufr_dcr_ros2  )
add_test(NAME test1 COMMAND test1)

install(TARGETS test1 DESTINATION bin)