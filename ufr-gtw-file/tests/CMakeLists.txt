add_executable(test_gtw_file ./test_file.c)
target_link_libraries(test_gtw_file ufr_gtw_file)
add_test(NAME test_gtw_file COMMAND test_gtw_file)
