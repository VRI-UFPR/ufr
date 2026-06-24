/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Felipe Bombardelli
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// ============================================================================
//  HEADER
// ============================================================================

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <ufr.h>
#include <ufr_test.h>

#include "test.h"

// ============================================================================
//  Tests
// ============================================================================

void test_decode_5i() {
    link_t* link = ufr_subscriber("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_dcr_msgpack_new);

    {
        const char send[] = {1,2,3,4,5};
        ufr_write(link, send, sizeof(send));
        UFR_TEST_TRUE( ufr_recv(link) );
        UFR_TEST_EQUAL( ufr_meta_pack_nitems(link), 5 );
        UFR_TEST_EQUAL( ufr_meta_item_nitems(link), 1 );
        for (int i=1; i<=5; i++) {
            int num;
            // assert( ufr_get_type(link) == 'i' );
            // UFR_TEST_EQUAL( ufr_get_nitems(link), 1 );
            // assert( ufr_get_raw_ptr(link) != NULL );
            UFR_TEST_EQUAL_STR( ufr_meta_item_mime(link), "number/u32" );
            UFR_TEST_EQUAL( ufr_get(link, "%d", &num), 1 );
            UFR_TEST_EQUAL( num, i );
        }
    }
}

void test_decode_3f() {
    link_t* link = ufr_subscriber("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_dcr_msgpack_new);

    {
        const char send[] = {0xCA, 0x41, 0x28, 0, 0, 0xCA, 0x41, 0xA2, 0, 0, 0xCA, 0x41, 0xF1, 0, 0};
        ufr_write(link, send, sizeof(send));
        UFR_TEST_TRUE( ufr_recv(link) );

        UFR_TEST_EQUAL( ufr_meta_pack_nitems(link), 3 );

        float num = 0;
        // assert( ufr_get_type(link) == 'f' );
        UFR_TEST_EQUAL_STR( ufr_meta_item_mime(link), "number/f32" );
        UFR_TEST_EQUAL( ufr_meta_item_nitems(link), 1 );
        UFR_TEST_EQUAL( ufr_get(link, "%f", &num), 1 );
        UFR_TEST_EQUAL_F32( num, 10.5 );

        // assert( ufr_get_type(link) == 'f' );
        UFR_TEST_EQUAL_STR( ufr_meta_item_mime(link), "number/f32" );
        UFR_TEST_EQUAL( ufr_meta_item_nitems(link), 1 );
        UFR_TEST_EQUAL( ufr_get(link, "%f", &num), 1 );
        UFR_TEST_EQUAL_F32( num, 20.25 );

        // assert( ufr_get_type(link) == 'f' );
        UFR_TEST_EQUAL_STR( ufr_meta_item_mime(link), "number/f32" );
        UFR_TEST_EQUAL( ufr_meta_item_nitems(link), 1 );
        UFR_TEST_EQUAL( ufr_get(link, "%f", &num), 1 );
        UFR_TEST_EQUAL_F32( num, 30.125 );
    }
}


void test_decode_2s() {
    link_t* link = ufr_subscriber("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_dcr_msgpack_new);

    {
        uint8_t send[] = {
            0xa6, 0x61, 0x62, 0x63, 0x31, 0x32, 0x33, 
            0xa6, 0x61, 0x62, 0x63, 0x33, 0x34, 0x35, 
            '\n'
        };
        ufr_write(link, (const char*) send, sizeof(send));
        ufr_recv(link);

        char str[32];
        // assert( ufr_get_type(link) == 's' );
        assert( ufr_get(link, "%s", str) == 1 );
        UFR_TEST_EQUAL_STR( str, "abc123" );

        // assert( ufr_get_type(link) == 's' );
        assert( ufr_get(link, "%s", str) == 1 );
        UFR_TEST_EQUAL_STR( str, "abc345" );
    }
}

void test_decode_file() {
    const char file_send[] = {
        0xc4, 0x1b, 0x74, 0x65, 0x78, 0x74, 0x2f, 0x70, 
        0x6c, 0x61, 0x69, 0x6e, 0x00, 0x01, 0x02, 0x03, 
        0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 
        0x0c, 0x0d, 0x0e, 0x0f, 0x10
    };

    link_t* link = ufr_subscriber("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_dcr_msgpack_new);
    ufr_write(link, (const char*) file_send, sizeof(file_send));
    UFR_TEST_TRUE( ufr_recv(link) );

    // Test metadata for package
    UFR_TEST_EQUAL_STR( ufr_meta_pack_mime(link), "text/plain" );
    UFR_TEST_EQUAL( ufr_meta_pack_nitems(link), 1 );
    UFR_TEST_EQUAL_U64( ufr_meta_pack_nbytes(link), sizeof(file_send));

    // Test metadata for item
    UFR_TEST_EQUAL_STR( ufr_meta_item_mime(link), "text/plain" );
    UFR_TEST_EQUAL( ufr_meta_item_nbytes(link), 16 );
    UFR_TEST_EQUAL( ufr_meta_item_nitems(link), 16 );

    // Test the data
    char* recv_mime;
    char* recv_data;
    int recv_nbytes;
    ufr_get_bin(link, &recv_mime, &recv_data, &recv_nbytes);
    UFR_TEST_EQUAL_STR( recv_mime, "text/plain" );
    for (int i=0; i<16; i++ ) {
        UFR_TEST_EQUAL( recv_data[i], file_send[13+i] );
    }
}



void test_decoded_array() {
    link_t* link = ufr_subscriber("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_dcr_msgpack_new);

    {
        uint8_t send[] = {0x95, 0x13, 0x14, 0x1e, 0x28, 0x32};
        ufr_write(link, (const char*) send, sizeof(send));
        ufr_recv(link);

        int num;
        // assert( ufr_get_type(link) == 'a' );
        UFR_TEST_OK( ufr_get_enter(link) );
        assert( ufr_get(link, "i", &num) == 1 );
        UFR_TEST_EQUAL( num, 0x13 );
        UFR_TEST_EQUAL( ufr_get(link, "%d", &num), 1 );
        UFR_TEST_EQUAL( num, 0x14 );
        UFR_TEST_EQUAL( ufr_get(link, "%d", &num), 1 );
        UFR_TEST_EQUAL( num, 0x1e );
        UFR_TEST_EQUAL( ufr_get(link, "%d", &num), 1 );
        UFR_TEST_EQUAL( num, 0x28 );
        UFR_TEST_EQUAL( ufr_get(link, "%d", &num), 1 );
        UFR_TEST_EQUAL( num, 0x32 );
        UFR_TEST_EQUAL( ufr_get(link, "%d", &num), 0 );
        UFR_TEST_EQUAL( num, 0 );
        UFR_TEST_OK( ufr_get_leave(link) );
    }
}

// ============================================================================
//  Tools
// ============================================================================


void show_encoder_bytes() {
printf("opa\n");
    link_t* link = ufr_publisher("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_enc_msgpack_new);

    // Envia os dados formatados
    // ufr_put(link, "%d %d %d %d %d\n", 10, 20, 30, 40, 50);

    const char file[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    ufr_put_bin(link, "text/plain", file, 16);
    ufr_put_eof(link);

    // Mostra os dados codificados
    uint8_t buffer[1024];
    ufr_recv(link);
    size_t read = ufr_read(link, (char*) buffer, 1024);
    for (int i=0; i<read; i++ ){
        printf("%x ", buffer[i]);
    }
    printf("\n");

    ufr_close(link);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test_decode_5i();
    test_decode_3f();
    test_decode_2s();
    test_decode_file();
    // show_encoder_bytes();

    ufr_test_print_result();
	return 0;
}