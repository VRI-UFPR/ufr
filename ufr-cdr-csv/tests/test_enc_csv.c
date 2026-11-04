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

int ufr_enc_csv_new(link_t* link);

// ============================================================================
//  Tests
// ============================================================================

void test_simple() {
    char buffer[1024];
    link_t* link = ufr_publisher("@new %p @coder %p @sep ; @buffer %p", 
        ufr_gtw_memory_new, ufr_enc_csv_new, buffer);

    // test 1
    {
        UFR_TEST_EQUAL_I32( ufr_put(link, "%d %d %d\n", 10, 20, 30), 4 );
        UFR_TEST_EQUAL_I32( link->sent, 9 );
        UFR_TEST_EQUAL_STR(buffer, "10;20;30\n");
    }


    // test 2
    {
        ufr_put(link, "%d %d %d\n", 30, 20, 10);
        UFR_TEST_EQUAL_I32( link->sent, 9 );
        UFR_TEST_EQUAL_STR(buffer, "30;20;10\n");

    }

    // test 3
    {
        ufr_put(link, "%s %d %d\n", "string com espaco", 8759834, -712345);
        UFR_TEST_EQUAL_I32( link->sent, 34 );
        UFR_TEST_EQUAL_STR(buffer, "string com espaco;8759834;-712345\n");
    }

    // test 4
    {
        for (int i=0; i<60; i++){
            ufr_put(link, "%d", i);
        }
        ufr_put(link, "\n");
        UFR_TEST_EQUAL_I32( link->sent, 170 );
        UFR_TEST_EQUAL_STR(buffer, "0;1;2;3;4;5;6;7;8;9;10;11;12;13;14;15;16;17;18;19;20;21;22;23;24;25;26;27;28;29;30;31;32;33;34;35;36;37;38;39;40;41;42;43;44;45;46;47;48;49;50;51;52;53;54;55;56;57;58;59\n");
    }

    // test 5
    {
        int vetor[50];
        for (int i=0; i<50; i++) {
            vetor[i] = i;
        }
        const int res = ufr_put(link, "%a:d:50\n", vetor);
        UFR_TEST_EQUAL_I32( res, 51 );
        UFR_TEST_EQUAL_I32( link->sent, 148 );
        UFR_TEST_EQUAL_STR(buffer, "#[#;0;1;2;3;4;5;6;7;8;9;10;11;12;13;14;15;16;17;18;19;20;21;22;23;24;25;26;27;28;29;30;31;32;33;34;35;36;37;38;39;40;41;42;43;44;45;46;47;48;49;#]#\n");
    }
    

    ufr_close(link);
}

void test_simple_2() {
    char buffer[1024];
    link_t* link = ufr_publisher("@new %p @coder %p @sep , @buffer %p", 
        ufr_gtw_memory_new, ufr_enc_csv_new, buffer);

    // test 1
    {
        ufr_put(link, "%d %d %d\n", 10, 20, 30);
        UFR_TEST_EQUAL_I32( link->sent, 9 );
        UFR_TEST_EQUAL_STR(buffer, "10,20,30\n");
    }

    // test 2
    {
        ufr_put(link, "%d %d %d\n", 30, 20, 10);
        UFR_TEST_EQUAL_I32( link->sent, 9 );
        UFR_TEST_EQUAL_STR(buffer, "30,20,10\n");
    }

    // test 3
    {
        ufr_put(link, "%s %d %d\n", "string com espaco", 8759834, -712345);
        UFR_TEST_EQUAL_I32( link->sent, 34 );
        UFR_TEST_EQUAL_STR(buffer, "string com espaco,8759834,-712345\n");
    }

    ufr_close(link);
}


// ============================================================================
//  Main
// ============================================================================

int main() {
    test_simple();
    test_simple_2();
    return 0;
}