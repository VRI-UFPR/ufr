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
#include <unistd.h>
#include <string.h>
#include <ufr.h>
#include "test.h"
#include <ufr_test.h>

// ============================================================================
//  Tests
// ============================================================================

void test_encode_3d() {
    char buffer[8];
    link_t* link = ufr_publisher("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_enc_msgpack_new);
    
    {
        ufr_put(link, "%d %d %d\n", 10, 20, 30);
        UFR_TEST_TRUE( ufr_recv(link) );
        UFR_TEST_EQUAL( ufr_read(link, buffer, sizeof(buffer)), 3 );
        UFR_TEST_EQUAL_I8( buffer[0], 10 );
        UFR_TEST_EQUAL_I8( buffer[1], 20 );
        UFR_TEST_EQUAL_I8( buffer[2], 30 );
    }

    // fim
    ufr_close(link);
}

void test_encode_s() {
    char buffer[8];
    link_t* link = ufr_publisher("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_enc_msgpack_new);

    {
        ufr_put(link, "%s\n", "hello");
        UFR_TEST_TRUE( ufr_recv(link) );
        UFR_TEST_EQUAL( ufr_read(link, buffer, sizeof(buffer)), 6 );
        UFR_TEST_EQUAL_I8( buffer[0], -91 );
        UFR_TEST_EQUAL_STR( &buffer[1], "hello" );
    }
}


void test_encode_dfs() {
    char buffer[16];
    link_t* link = ufr_publisher("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_enc_msgpack_new);
    
    {
        ufr_put(link, "%d %f %s\n", 10, 20.525, "texto");
        UFR_TEST_TRUE( ufr_recv(link) );
        UFR_TEST_EQUAL_I8( ufr_read(link, buffer, sizeof(buffer)), 12 );
        UFR_TEST_EQUAL_I8( buffer[0], 0x0a );
        UFR_TEST_EQUAL_I8( buffer[1], 0xca );
        UFR_TEST_EQUAL_I8( buffer[2], 0x41 );
        UFR_TEST_EQUAL_I8( buffer[3], 0xa4 );
        UFR_TEST_EQUAL_I8( buffer[4], 0x33 );
        UFR_TEST_EQUAL_I8( buffer[5], 0x33 );
        UFR_TEST_EQUAL_I8( buffer[6], 0xa5 );
        UFR_TEST_EQUAL_I8( buffer[7], 0x74 );
        UFR_TEST_EQUAL_I8( buffer[8], 0x65 );
        UFR_TEST_EQUAL_I8( buffer[9], 0x78 );
        UFR_TEST_EQUAL_I8( buffer[10], 0x74 );
        UFR_TEST_EQUAL_I8( buffer[11], 0x6f );
    }
}


void test_encoder_array() {
    link_t* link = ufr_publisher("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_enc_msgpack_new);
    
    {
        int vet[5] = {20,21,22,23,24};
        // ufr_put(link, "ai\n", 5, vet);
        ufr_put_pi32(link, vet, 5);
        ufr_put(link, "\n");
        uint8_t buffer[8];
        assert( ufr_read(link, (char*) buffer, sizeof(buffer)) == 7 );
        
        // assert( ufr_read(link, buffer, sizeof(buffer)) == 5 );
        assert( buffer[0] == 0x95 );
        assert( buffer[1] == 20 );
        assert( buffer[2] == 21 );
        assert( buffer[3] == 22 );
        assert( buffer[4] == 23 );
        assert( buffer[5] == 24 );
    }

    ufr_close(link);
    printf("encoded 1 array - OK\n");
}


// ============================================================================
//  Tools
// ============================================================================

void show_encoder_bytes() {
    link_t* link = ufr_publisher("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_enc_msgpack_new);

    ufr_put(link, "%d %f %s\n", 10, 20.525, "texto");

    /*ufr_put_enter(link, 5);
    ufr_put(link, "%d %d %d %d %d", 10, 20, 30, 40, 50);
    ufr_put_leave(link);
    ufr_put(link, "\n");*/

    /* ufr_enter_array(link, 3);
    for (int i=0; i<3; i++) {
        ufr_put(link, "i", i);
    }
    ufr_leave_array(link);
    ufr_put(link, "\n");*/

    ufr_recv(link);
    uint8_t buffer[1024];
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
    test_encode_3d();
    test_encode_s();
    test_encode_dfs();
    // test_encoder_array();
    // show_encoder_bytes();
    ufr_test_print_result();
    return 0;
}