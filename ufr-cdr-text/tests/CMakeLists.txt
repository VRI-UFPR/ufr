add_executable(test_enc_text ./test_enc_text.c)
target_link_libraries(test_enc_text ufr_enc_text)
add_test(NAME test_enc_text COMMAND test_enc_text)

# add_test(NAME test_valgrind_ecr_std COMMAND valgrind test_ecr_std)

add_executable(test_dcr_text ./test_dcr_text.c)
target_link_libraries(test_dcr_text ufr_dcr_text)
add_test(NAME test_dcr_text COMMAND test_dcr_text)

# add_test(NAME test_valgrind_dcr_std COMMAND valgrind test_dcr_std)