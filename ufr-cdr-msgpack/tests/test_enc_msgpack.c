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
#include <ufr_test.h>

#include "test.h"

// ============================================================================
//  Tests
// ============================================================================

void test() {
    uint8_t buffer[1024];
    link_t* link = ufr_publisher("@new %p @coder %p @buffer %p", 
        ufr_gtw_memory_new, ufr_enc_msgpack_new, buffer);
    
    // teste 1
    {
        const uint8_t correto[] = {0x83, 0xa2, 0x76, 0x30, 0xa, 0xa2, 0x76, 0x31, 0x14, 0xa2, 0x76, 0x32, 0x1e};
        ufr_put(link, "%d %d %d\n", 10, 20, 30);
        UFR_TEST_EQUAL_U64 ( link->sent, sizeof(correto) );
        for (int i=0; i<sizeof(correto); i++) {
            UFR_TEST_EQUAL_U8( buffer[i], correto[i] );
        }
    }

    // teste 2
    {
        const uint8_t correto[] = {0x81, 0xa2, 0x76, 0x30, 0xa5, 0x68, 0x65, 0x6c, 0x6c, 0x6f};
        ufr_put(link, "%s\n", "hello");
        UFR_TEST_EQUAL_U64 ( link->sent, sizeof(correto) );
        for (int i=0; i<sizeof(correto); i++) {
            UFR_TEST_EQUAL_U8( buffer[i], correto[i] );
        }
    }

    // teste 3
    {
        const uint8_t correto[] = {0x83, 0xa2, 0x76, 0x30, 0xa, 0xa2, 0x76, 0x31, 0xca, 0x41, 0xa4, 0x33, 0x33, 0xa2, 0x76, 0x32, 0xa5, 0x74, 0x65, 0x78, 0x74, 0x6f};
        ufr_put(link, "%d %f %s\n", 10, 20.525, "texto");
        UFR_TEST_EQUAL_U64 ( link->sent, sizeof(correto) );
        for (int i=0; i<sizeof(correto); i++) {
            UFR_TEST_EQUAL_U8( buffer[i], correto[i] );
        }
    }

    // teste 4
    {
        const uint8_t correto[] = {0x81, 0xa2, 0x76, 0x30, 0x95, 0x14, 0x15, 0x16, 0x17, 0x18};
        const int vet[5] = {20,21,22,23,24};
        ufr_put(link, "%a:d:5\n", vet);
        UFR_TEST_EQUAL_U64 ( link->sent, sizeof(correto) );
        for (int i=0; i<sizeof(correto); i++) {
            UFR_TEST_EQUAL_U8( buffer[i], correto[i] );
        }
    }

    // teste 5
    {
        const uint8_t correto[] = {
            0x84, 0xa4, 0x6e, 0x61, 0x6d, 0x65, 0xa6, 0x66, 0x65, 0x6c, 0x69, 0x70, 
            0x65, 0xa5, 0x73, 0x63, 0x6f, 0x72, 0x65, 0xca, 0x41, 0x28, 0x0, 0x0, 0xa3, 0x61, 0x67, 0x65, 
            0x1e, 0xa5, 0x76, 0x65, 0x74, 0x6f, 0x72, 0x95, 0x14, 0x15, 0x16, 0x17, 0x18
        };
        const int vet[5] = {20,21,22,23,24};
        ufr_put(link, "name=%s score=%f age=%d vetor=%a:d:5\n", "felipe", 10.5, 30, vet);
        UFR_TEST_EQUAL_U64 ( link->sent, sizeof(correto) );
        for (int i=0; i<sizeof(correto); i++) {
            UFR_TEST_EQUAL_U8( buffer[i], correto[i] );
        }
    }

    // fim
    ufr_close(link);
}


// ============================================================================
//  Tools
// ============================================================================

void show_encoder_bytes() {
    uint8_t buffer[1024];
    link_t* link = ufr_publisher("@new %p @coder %p @buffer %p", 
        ufr_gtw_memory_new, ufr_enc_msgpack_new, buffer);

    int vet[5] = {20,21,22,23,24};
    ufr_put(link, "name=%s score=%f age=%d vetor=%a:d:5\n", "felipe", 10.5, 30, vet);
    // ufr_put(link, "%d %f %s\n", 10, 20.525, "texto");
    // ufr_put(link, "%d %d %d\n", 10, 20, 30);
    // ufr_put(link, "%s\n", "hello");
    
    printf("const uint8_t correto[] = {");
    for (int i=0; i<link->sent; i++ ){
        printf("0x%x, ", buffer[i]);
    }
    printf("};\n");

    ufr_close(link);
}


// ============================================================================
//  Main
// ============================================================================

int main() {
    // show_encoder_bytes();
    test();
    return 0;
}