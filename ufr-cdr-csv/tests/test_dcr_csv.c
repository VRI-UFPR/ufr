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

int ufr_dcr_csv_new(link_t* link);

// ============================================================================
//  Tests
// ============================================================================

void test_simple() {
    link_t* link = ufr_subscriber("@new %p @coder %p @sep ;", 
        ufr_gtw_memory_new, ufr_dcr_csv_new);

    // test 1
    {
        int a=0,b=0,c=0;
        const char buffer[] = "10;20;30\n";
        ufr_gtw_write_i8(link, buffer, strlen(buffer));
        ufr_get(link, "> %d %d %d", &a, &b, &c);
        UFR_TEST_EQUAL_I32(a, 10);
        UFR_TEST_EQUAL_I32(b, 20);
        UFR_TEST_EQUAL_I32(c, 30);
        printf("Teste 1.1 - OK\n");
    }

    // test 2
    {
        float a=0,b=0,c=0;
        const char* buffer = "10.5;20.125;30.5\n";
        ufr_gtw_write_i8(link, buffer, strlen(buffer));
        ufr_get(link, "> %f %f %f", &a, &b, &c);
        UFR_TEST_EQUAL_F32(a, 10.5);
        UFR_TEST_EQUAL_F32(b, 20.125);
        UFR_TEST_EQUAL_F32(c, 30.5);
        printf("Teste 1.2 - OK\n");
    }

    // test 3
    {
        const char* buffer = "0;1;2;3;4;5;6;7;8;9;10;11;12;13;14;15;16;17;18;19;20;21;22;23;24;25;26;27;28;29;30;31;32;33;34;35;36;37;38;39;40;41;42;43;44;45;46;47;48;49;50;51;52;53;54;55;56;57;58;59\n";
        ufr_gtw_write_i8(link, buffer, strlen(buffer));
        
        const int res = ufr_get(link, ">");
        UFR_TEST_EQUAL(res,1);
        for (int i=0; i<60; i++) {
            int num;
            const int res2 = ufr_get(link, "%d", &num);
            UFR_TEST_EQUAL(res2,1);
            UFR_TEST_EQUAL(num,i);
        }
        printf("Teste 1.3 - OK\n");
    }

    // test 4
    {
        const char* buffer = "#[#;0;1;2;3;4;5;6;7;8;9;10;11;12;13;14;15;16;17;18;19;20;21;22;23;24;25;26;27;28;29;30;31;32;33;34;35;36;37;38;39;40;41;42;43;44;45;46;47;48;49;#]#\n"
        const int res = ufr_get(link, "%a:d:50\n", vetor);
        UFR_TEST_EQUAL_I32( res, 51 );
        UFR_TEST_EQUAL_I32( link->sent, 148 );
        UFR_TEST_EQUAL_STR(buffer, "");
    }

    ufr_close(link);
}

void test_simple_2() {
    link_t* link = ufr_subscriber("@new %p @coder %p @sep ,", 
        ufr_gtw_posix_new_pipe, ufr_dcr_csv_new);

    // test 1
    {
        int a=0,b=0,c=0;
        const char* buffer = "10,20,30\n";
        ufr_gtw_write_i8(link, buffer, strlen(buffer));
        ufr_get(link, "> %d %d %d", &a, &b, &c);
        UFR_TEST_EQUAL_I32(a, 10);
        UFR_TEST_EQUAL_I32(b, 20);
        UFR_TEST_EQUAL_I32(c, 30);
    }

    // test 2
    {
        float a=0,b=0,c=0;
        const char* buffer = "10.5,20.125,30.5\n";
        ufr_gtw_write_i8(link, buffer, strlen(buffer));
        ufr_get(link, "> %f %f %f", &a, &b, &c);
        UFR_TEST_EQUAL_F32(a, 10.5);
        UFR_TEST_EQUAL_F32(b, 20.125);
        UFR_TEST_EQUAL_F32(c, 30.5);
    }

    // test 3
    {
        char text[512];
        const char* buffer = "abcde\n";
        ufr_gtw_write_i8(link, buffer, strlen(buffer));
        ufr_get(link, "> %s", text);
        UFR_TEST_EQUAL_STR(text, "abcde");
    }

    // test 4 - max size on buffer
    /*
    {
        char text[8];
        const char* buffer = "aaabbbcccddd\n";
        ufr_write(&link, buffer, strlen(buffer));
        ufr_get(&link, "^s", text);
        assert( strcmp(text, "aaabbbc") == 0 );
    }
    */

    ufr_close(link);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test_simple();
    test_simple_2();
    ufr_test_print_result();
	return 0;
}