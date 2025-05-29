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

int ufr_dcr_text_new(link_t* link);

// ============================================================================
//  Tests
// ============================================================================

void test_simple() {
    link_t link = ufr_subscriber("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_dcr_text_new);

    // test 1 - simples string
    {
        char recv[512];
        const char* send = "10 20 30";
        ufr_write(&link, send, strlen(send));
        ufr_get(&link, "^s", &recv);
        UFR_TEST_EQUAL_STR(recv, send);
    }

    // test 2 - string with new lines and special caracters
    {
        char recv[512];
        const char* send = "10 20 30\n20 30 50 60\n40 50 70\nççç&5%";
        ufr_write(&link, send, strlen(send));
        ufr_get(&link, "^s", &recv);
        UFR_TEST_EQUAL_STR(recv, send);
    }

    // test 3 - many sendings (COM ERROS)
    /* {
        char recv[1024];
        char send1[1024];
        for (int i=0; i<1024; i++){
            send1[i] = 'a';
        }
        ufr_write(&link, send1, 1024);
        ufr_get(&link, "^s", &recv);
        char correct1[512];
        for (int i=0; i<512; i++){
            correct1[i] = 'a';
        }
        correct1[511] = '\0';
        UFR_TEST_EQUAL_STR(recv, send1);
    }*/

    ufr_close(&link);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test_simple();
    ufr_test_print_result();
	return 0;
}