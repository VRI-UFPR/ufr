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
#include <pthread.h>

int ufr_gtw_zmq_new_topic(link_t* link, const int type);

// ============================================================================
//  Test 1
// ============================================================================

void test1_publisher() {
    printf("Starting publisher\n");
    link_t link = ufr_publisher("@new %p @host 127.0.0.1 @port 3000", ufr_gtw_zmq_new_topic);
    ufr_put(&link, "iii\n", 10, 20, 30);
    ufr_close(&link);
}

void* test1_subscriber(void* ptr) {
    printf("Starting subscriber\n");
    link_t link = ufr_subscriber("@new %p @host 127.0.0.1 @port 3000", ufr_gtw_zmq_new_topic);

    int a=0,b=0,c=0;
    ufr_get(&link, "^iii", &a, &b, &c);
    UFR_TEST_EQUAL_I32( a, 10 );
    UFR_TEST_EQUAL_I32( b, 20 );
    UFR_TEST_EQUAL_I32( c, 30 );

    ufr_close(&link);
    return NULL;
}

void test1() {
    pthread_t thread_sub;
    pthread_create(&thread_sub, NULL, test1_subscriber, NULL);
    test1_publisher();
    pthread_join(thread_sub, NULL);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test1();
    ufr_test_print_result();
	return 0;
}