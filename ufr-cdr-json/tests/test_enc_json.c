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
//  Main
// ============================================================================



int main() {
    char buffer[1024];
    link_t* link = ufr_publisher("@new %p @coder %p @buffer %p", 
        ufr_gtw_memory_new, ufr_enc_msgpack_new, buffer);
    
    // Teste 1
    {
        ufr_put(link, "%d %d %d\n", 10, 20, 30);
        UFR_TEST_EQUAL_STR( buffer, "{'v0': 10, 'v1': 20, 'v2': 30}" );
    }

    // Teste 2
    {
        ufr_put(link, "%f %d %f %d\n", 10.525, 20, 3.141592, 30);
        UFR_TEST_EQUAL_STR( buffer, "{'v0': 10.525000, 'v1': 20, 'v2': 3.141592, 'v3': 30}" );
    }

    // Teste 3
    {
        ufr_put(link, "%f %d %s %f %d %s\n", 10.525, 20, "felipe", 3.141592, 30, "opa");
        UFR_TEST_EQUAL_STR( buffer, "{'v0': 10.525000, 'v1': 20, 'v2': 'felipe', 'v3': 3.141592, 'v4': 30, 'v5': 'opa'}" );
    }

    // Teste 4
    {
        int vetor[4] = {10,20,30,40};
        ufr_put(link, "%a:d:4\n", vetor);
        UFR_TEST_EQUAL_STR( buffer, "{'v0': [10,20,30,40]}" );
    }

    // Teste 5
    {
        float vetor[4] = {10.5,20.25,30.1234,40.125004};
        ufr_put(link, "%a:f:4\n", vetor);
        UFR_TEST_EQUAL_STR( buffer, "{'v0': [10.500000,20.250000,30.123400,40.125004]}" );
    }

    // Teste 6
    {
        int vetor1[4] = {10,20,30,40};
        float vetor2[4] = {10.5,20.25,30.1234,40.125004};
        ufr_put(link, "%a:d:4 %a:f:4 %s\n", vetor1, vetor2, "opa");
        UFR_TEST_EQUAL_STR( buffer, "{'v0': [10,20,30,40], 'v1': [10.500000,20.250000,30.123400,40.125004], 'v2': 'opa'}" );
    }

    // fim
    ufr_close(link);
    return 0;
}