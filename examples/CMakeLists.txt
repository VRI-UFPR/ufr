# find_package(OpenCV)

add_executable(example_publisher publisher.c)
target_link_libraries(example_publisher  ufr)
set_target_properties(example_publisher PROPERTIES OUTPUT_NAME "publisher")
install(TARGETS example_publisher DESTINATION examples)

add_executable(example_subscriber subscriber.c)
target_link_libraries(example_subscriber  ufr)
set_target_properties(example_subscriber PROPERTIES OUTPUT_NAME "subscriber")
install(TARGETS example_subscriber DESTINATION examples)

add_executable(example_client client.c)
target_link_libraries(example_client  ufr)
set_target_properties(example_client PROPERTIES OUTPUT_NAME "client")
install(TARGETS example_client DESTINATION examples)

add_executable(example_server server.c)
target_link_libraries(example_server  ufr)
set_target_properties(example_server PROPERTIES OUTPUT_NAME "server")
install(TARGETS example_server DESTINATION examples)



### 