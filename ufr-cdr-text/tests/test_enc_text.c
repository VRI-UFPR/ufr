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

int ufr_enc_text_new(link_t* link);

// ============================================================================
//  Tests
// ============================================================================

void test_simple() {
    link_t link = ufr_publisher("@new %p @coder %p", 
        ufr_gtw_posix_new_pipe, ufr_enc_text_new);

    // test 1
    {
        char buffer[128];
        UFR_TEST_EQUAL_I32( ufr_put(&link, "iii\n", 10, 20, 30), 3 );
        UFR_TEST_OK( ufr_recv(&link) );
        const int nbytes = ufr_read(&link, buffer, sizeof(buffer));
        UFR_TEST_EQUAL_I32( nbytes, 9 );
        buffer[nbytes] = '\0';
        UFR_TEST_EQUAL_STR(buffer, "10 20 30\n");
    }

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