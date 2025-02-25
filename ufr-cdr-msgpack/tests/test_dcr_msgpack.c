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

void test_decoded_5i() {
    link_t link = ufr_subscriber("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_dcr_msgpack_new);

    {
        char send[] = {1,2,3,4,5,'\n'};
        ufr_write(&link, send, sizeof(send));
        ufr_recv(&link);
        for (int i=1; i<=5; i++) {
            int num;
            assert( ufr_get_type(&link) == 'i' );
            UFR_TEST_EQUAL( ufr_get_nitems(&link), 1 );
            // assert( ufr_get_raw_ptr(&link) != NULL );
            UFR_TEST_EQUAL( ufr_get(&link, "i", &num), 1 );
            UFR_TEST_EQUAL( num, i );
        }
    }
}

void test_decoded_3f() {
    link_t link = ufr_subscriber("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_dcr_msgpack_new);

    {
        char send[] = {0xCA, 0x41, 0x28, 0, 0, 0xCA, 0x41, 0xA2, 0, 0, 0xCA, 0x41, 0xF1, 0, 0, '\n'};
        ufr_write(&link, send, sizeof(send));
        ufr_recv(&link);

        float num;
        assert( ufr_get_type(&link) == 'f' );
        UFR_TEST_EQUAL( ufr_get(&link, "f", &num), 1 );
        UFR_TEST_EQUAL_F32( num, 10.5 );

        assert( ufr_get_type(&link) == 'f' );
        UFR_TEST_EQUAL( ufr_get(&link, "f", &num), 1 );
        UFR_TEST_EQUAL_F32( num, 20.25 );

        assert( ufr_get_type(&link) == 'f' );
        UFR_TEST_EQUAL( ufr_get(&link, "f", &num), 1 );
        UFR_TEST_EQUAL_F32( num, 30.125 );
    }
}


void test_decoded_2s() {
    link_t link = ufr_subscriber("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_dcr_msgpack_new);

    {
        uint8_t send[] = {
            0xa6, 0x61, 0x62, 0x63, 0x31, 0x32, 0x33, 
            0xa6, 0x61, 0x62, 0x63, 0x33, 0x34, 0x35, 
            '\n'
        };
        ufr_write(&link, send, sizeof(send));
        ufr_recv(&link);

        char str[32];
        assert( ufr_get_type(&link) == 's' );
        assert( ufr_get(&link, "s", str) == 1 );
        UFR_TEST_EQUAL_STR( str, "abc123" );

        assert( ufr_get_type(&link) == 's' );
        assert( ufr_get(&link, "s", str) == 1 );
        UFR_TEST_EQUAL_STR( str, "abc345" );
    }
}

void test_decoded_array() {
    link_t link = ufr_subscriber("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_dcr_msgpack_new);

    {
        uint8_t send[] = {0x95, 0x13, 0x14, 0x1e, 0x28, 0x32, '\n'};
        ufr_write(&link, send, sizeof(send));
        ufr_recv(&link);

        int num;
        assert( ufr_get_type(&link) == 'a' );
        UFR_TEST_OK( ufr_get_enter(&link) );
        assert( ufr_get(&link, "i", &num) == 1 );
        UFR_TEST_EQUAL( num, 0x13 );
        UFR_TEST_EQUAL( ufr_get(&link, "i", &num), 1 );
        UFR_TEST_EQUAL( num, 0x14 );
        UFR_TEST_EQUAL( ufr_get(&link, "i", &num), 1 );
        UFR_TEST_EQUAL( num, 0x1e );
        UFR_TEST_EQUAL( ufr_get(&link, "i", &num), 1 );
        UFR_TEST_EQUAL( num, 0x28 );
        UFR_TEST_EQUAL( ufr_get(&link, "i", &num), 1 );
        UFR_TEST_EQUAL( num, 0x32 );
        UFR_TEST_EQUAL( ufr_get(&link, "i", &num), 0 );
        UFR_TEST_EQUAL( num, 0 );
        UFR_TEST_OK( ufr_get_leave(&link) );
    }
}


void test1() {
    link_t link = ufr_subscriber("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_dcr_msgpack_new);

    {
        int recv[8];
        char send[] = {1,2,3,4,5,'\n'};
        ufr_write(&link, send, 7);
        ufr_get(&link, "^i", recv);
        for (int i=0; i<1; i++) {
            printf("%d ", recv[i]);
        }
        printf("\n");
    }

    ufr_close(&link);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test_decoded_5i();
    test_decoded_3f();
    test_decoded_2s();
    // test_decoded_array();

    ufr_test_print_result();
	return 0;
}