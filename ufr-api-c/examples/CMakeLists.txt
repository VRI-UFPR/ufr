add_executable(ufr_client client.c)
target_link_libraries(ufr_client ufr)

add_executable(ufr_server server.c)
target_link_libraries(ufr_server ufr)

add_executable(ufr_publisher publisher.c)
target_link_libraries(ufr_publisher ufr)

add_executable(ufr_subscriber subscriber.c)
target_link_libraries(ufr_subscriber ufr)