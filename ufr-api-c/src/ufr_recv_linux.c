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
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
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
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "ufr.h"

// ============================================================================
//  UFR RECV
// ============================================================================

int ufr_recv_sy2(link_t* link0, link_t* link1, int time_ms) {
    // check the 2 links
    uint8_t i = 0;
    const uint8_t max = 10;
    const int time_us = time_ms * 1000/max;
    for(; i<max; i++) {
        if ( ufr_recv_async(link0) == UFR_OK ) {
            goto second;
        }

        if ( ufr_recv_async(link1) == UFR_OK ) {
            goto first;
        }

        usleep(time_us);
    }
    goto timeout;

first:
    do {
        usleep(time_us);
        if ( ufr_recv_async(link0) == UFR_OK ) {
            goto success;
        }
    } while ( i < max );
    goto timeout;

second:
    do {
        usleep(time_us);
        if ( ufr_recv_async(link1) == UFR_OK ) {
            goto success;
        }
    } while ( i < max );
    goto timeout;

timeout:
    return -1;

success:
    return UFR_OK;
}

int ufr_recv_as2(link_t* link0, link_t* link1, int time_ms) {
    // check the 2 links
    uint8_t i = 0;
    const uint8_t max = 4;
    const int time_us = time_ms * 1000/max;
    for(; i<max; i++) {
        if ( ufr_recv_async(link0) == UFR_OK ) {
            return 0;
        }
        if ( ufr_recv_async(link1) == UFR_OK ) {
            return 1;
        }
        usleep(time_us);
    }
    return -1;
}

int ufr_recv_as3(link_t* link0, link_t* link1, link_t* link2, int time_ms) {
    // check the 2 links
    uint8_t i = 0;
    const uint8_t max = 4;
    const int time_us = time_ms * 1000/max;
    for(; i<max; i++) {
        if ( ufr_recv_async(link0) == UFR_OK ) {
            return 0;
        }
        if ( ufr_recv_async(link1) == UFR_OK ) {
            return 1;
        }
        if ( ufr_recv_async(link2) == UFR_OK ) {
            return 2;
        }
        usleep(time_us);
    }
    return -1;
}

