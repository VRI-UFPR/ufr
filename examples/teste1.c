/* BSD 2-Clause License
 * 
 * Copyright (c) 2024, Visao Robotica e Imagem (VRI)
 *  - Felipe Bombardelli <felipebombardelli@gmail.com>
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
 * DISCLAIMED. IN NO EVENT SHALL aTHE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
	
// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <ufr.h>
#include <unistd.h>
#include <pthread.h>
#include <ufr_test.h>

void* test_sub(void* parameter) {
    // begin
    const char* link_params = (const char*) parameter;
    link_t lnk = ufr_subscriber(link_params);
    UFR_TEST_TRUE( ufr_link_is_subscriber(&lnk) );
    UFR_TEST_FALSE( ufr_link_is_publisher(&lnk) );
    UFR_TEST_FALSE( ufr_link_is_client(&lnk) );
    UFR_TEST_FALSE( ufr_link_is_server(&lnk) );

    // test1 : simple primitives (int positive,int negative,float,string)
    if ( ufr_recv(&lnk) ) {
        // test metadata from package
        {
            int pack_nbytes = ufr_meta_pack_nbytes(&lnk);
            UFR_TEST_EQUAL(pack_nbytes, 25);
            int pack_nitems = ufr_meta_pack_nitems(&lnk);
            UFR_TEST_EQUAL(pack_nitems, 6);
            const char* pack_mime = ufr_meta_pack_mime(&lnk);
            UFR_TEST_EQUAL_STR(pack_mime, "list");
        }

        // test1A
        {
            int val;
            int nbytes = ufr_meta_item_nbytes(&lnk);
            UFR_TEST_EQUAL(nbytes, 4);
            int nitems = ufr_meta_item_nitems(&lnk);
            UFR_TEST_EQUAL(nitems, 1);
            const char* mime = ufr_meta_item_mime(&lnk);
            UFR_TEST_EQUAL_STR(mime, "number/u32");
            ufr_get(&lnk, "%d", &val);
            UFR_TEST_EQUAL_I32(val, 0);
        }

        // test1B
        {
            int val;
            int nbytes = ufr_meta_item_nbytes(&lnk);
            UFR_TEST_EQUAL(nbytes, 4);
            int nitems = ufr_meta_item_nitems(&lnk);
            UFR_TEST_EQUAL(nitems, 1);
            const char* mime = ufr_meta_item_mime(&lnk);
            UFR_TEST_EQUAL_STR(mime, "number/u32");
            ufr_get(&lnk, "%d", &val);
            UFR_TEST_EQUAL_I32(val, 2147483647);
        }

        // test1C
        {
            int val;
            int nbytes = ufr_meta_item_nbytes(&lnk);
            UFR_TEST_EQUAL(nbytes, 4);
            int nitems = ufr_meta_item_nitems(&lnk);
            UFR_TEST_EQUAL(nitems, 1);
            const char* mime = ufr_meta_item_mime(&lnk);
            UFR_TEST_EQUAL_STR(mime, "number/i32");
            ufr_get(&lnk, "%d", &val);
            UFR_TEST_EQUAL_I32(val, -2147483648);
        }

        // test1D
        {
            float val;
            int nbytes = ufr_meta_item_nbytes(&lnk);
            UFR_TEST_EQUAL(nbytes, 4);
            int nitems = ufr_meta_item_nitems(&lnk);
            UFR_TEST_EQUAL(nitems, 1);
            const char* mime = ufr_meta_item_mime(&lnk);
            UFR_TEST_EQUAL_STR(mime, "number/f32");
            ufr_get(&lnk, "%f", &val);
            UFR_TEST_EQUAL_F32(val, 1.625);
        }

        // test1E
        {
            float val;
            int nbytes = ufr_meta_item_nbytes(&lnk);
            UFR_TEST_EQUAL(nbytes, 4);
            int nitems = ufr_meta_item_nitems(&lnk);
            UFR_TEST_EQUAL(nitems, 1);
            const char* mime = ufr_meta_item_mime(&lnk);
            UFR_TEST_EQUAL_STR(mime, "number/f32");
            ufr_get(&lnk, "%f", &val);
            UFR_TEST_EQUAL_F32(val, -1.625);
        }

        // test1F
        {
            char val[1024];
            int nbytes = ufr_meta_item_nbytes(&lnk);
            UFR_TEST_EQUAL(nbytes, 3);
            int nitems = ufr_meta_item_nitems(&lnk);
            UFR_TEST_EQUAL(nitems, 3);
            const char* mime = ufr_meta_item_mime(&lnk);
            UFR_TEST_EQUAL_STR(mime, "text/plain");
            ufr_get(&lnk, "%s", &val);
            UFR_TEST_EQUAL_STR(val, "opa");
        }
    }

    // Test2 : simple string
    if ( ufr_recv(&lnk) ) {
        // Test Metadata from Package
        {
            int pack_nbytes = ufr_meta_pack_nbytes(&lnk);
            UFR_TEST_EQUAL(pack_nbytes, 4136);
            int pack_nitems = ufr_meta_pack_nitems(&lnk);
            UFR_TEST_EQUAL(pack_nitems, 4);
            const char* pack_mime = ufr_meta_pack_mime(&lnk);
            UFR_TEST_EQUAL_STR(pack_mime, "list");
        }

        // Test the 4 x %s
        for (int j=0; j<4; j++) {
            // test metadata from item
            {
                int nbytes = ufr_meta_item_nbytes(&lnk);
                UFR_TEST_EQUAL(nbytes, 1031);
                int nitems = ufr_meta_item_nitems(&lnk);
                UFR_TEST_EQUAL(nitems, 1031);
                const char* mime = ufr_meta_item_mime(&lnk);
                UFR_TEST_EQUAL_STR(mime, "text/plain");
            }

            // test data from item
            {
                char buffer[1024];
                ufr_get(&lnk, "%s", buffer);
                UFR_TEST_EQUAL_I32(strlen(buffer), 1023);
                char real[1024];
                for (int i=0; i<1024; i++) {
                    real[i] = 'a';
                }
                real[1023] = '\0';
                UFR_TEST_EQUAL_STR(buffer, real); 
            }
        }
    }

    // test3
    {
        if ( ufr_recv(&lnk) ) {
            const int nbytes1 = ufr_meta_item_nbytes(&lnk);
            UFR_TEST_EQUAL(nbytes1, 62301); // arrumar, retirar o mime

            char* ptr_mime;
            char* ptr_data;
            int nbytes2;
            UFR_TEST_OK ( ufr_get_bin(&lnk, &ptr_mime, &ptr_data, &nbytes2) );
            UFR_TEST_EQUAL( nbytes2, 62291 );
            UFR_TEST_EQUAL_STR(ptr_mime, "image/jpeg");
        }
    }

    // test3 :
    /*
    if ( ufr_recv(&lnk) ) {
        for (int i=0; i<10000; i++) {
            int val;
            ufr_get(&lnk, "%d", &val);
            UFR_TEST_EQUAL(val, i);

            // ufr_meta_item_type(&lnk); -> 'd'
            // ufr_meta_item_mime(&lnk); -> "number/i32"
            // ufr_meta_item_nbytes(&lnk)

            // ufr_meta_pack_mime(&lnk); -> "number/i32"
            // ufr_meta_pack_nitems(&lnk)
            // ufr_meta_pack_hostname(&lnk)
        }
    }

    // test4 :
    if ( ufr_recv(&lnk) == UFR_OK ) {
        for (int i=0; i<10000; i++) {
            float val;
            ufr_get(&lnk, "%f", &val);
            UFR_TEST_EQUAL_F32(val, i*1.5);
        }
    }
    */

    return NULL;
}


void test(const char* link_params) {
    printf("Teste1 : %s\n", link_params);
    pthread_t thread_id;
    pthread_create(&thread_id, NULL, test_sub, (void*) link_params);

    //
    link_t lnk = ufr_publisher(link_params);
    UFR_TEST_FALSE( ufr_link_is_subscriber(&lnk) );
    UFR_TEST_TRUE( ufr_link_is_publisher(&lnk) );
    UFR_TEST_FALSE( ufr_link_is_client(&lnk) );
    UFR_TEST_FALSE( ufr_link_is_server(&lnk) );
    sleep(1);

    // Test1 : send simple data
    {
        ufr_put(&lnk, "%d %d %d %f %f %s\n", 0,  2147483647, -2147483648, 1.625, -1.625, "opa");
    }

    // Test2 : send 4 string with 1032 bytes
    {
        char buffer[1032];
        for (int i=0; i<1031; i++) {
            buffer[i] = 'a';
        }
        buffer[1031] = '\0';
        ufr_put(&lnk, "%s %s %s %s\n", buffer, buffer, buffer, buffer);
    }

    // Test3 : send 
    {
        static char buffer[1024*1024];
        FILE* fd = fopen("aaa.jpg", "rb");
        const int nbytes = fread(buffer, 1, 1024*1024, fd);
        ufr_put_bin(&lnk, "image/jpeg", buffer, nbytes);
        ufr_send(&lnk);
        fclose(fd);
    }

    /*
    // test3 :
    for (int i=0; i<10000; i++) {
        ufr_put(&lnk, "%d", i);
    }
    ufr_send(&lnk);

    // test4
    for (int i=0; i<10000; i++) {
        ufr_put(&lnk, "%f", i*1.5);
    }
    ufr_send(&lnk);
    */

    pthread_join(thread_id, NULL);
}





// ============================================================================
//  Main
// ============================================================================

int main() {
    test("@new mqtt @coder msgpack @topic teste");
    ufr_test_print_result();
    return 0;
}