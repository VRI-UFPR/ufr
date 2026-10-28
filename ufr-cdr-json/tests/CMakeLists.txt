add_executable(test_enc_json ./test_enc_json.c)
target_link_libraries(test_enc_json ufr_cdr_json)
add_test(NAME test_enc_json COMMAND test_enc_json)

# add_executable(test_dcr_msgpack ./test_dcr_msgpack.c)
# target_link_libraries(test_dcr_msgpack ufr_cdr_msgpack)
# add_test(NAME test_dcr_msgpack COMMAND test_dcr_msgpack)