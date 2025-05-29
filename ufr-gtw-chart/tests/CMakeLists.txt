add_executable(test_gtw_chart ./main.c)
target_link_libraries(test_gtw_chart ufr_gtw_chart m)
add_test(NAME test_gtw_chart COMMAND test_gtw_chart)
