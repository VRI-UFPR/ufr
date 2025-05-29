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
#include <dlfcn.h>
#include <errno.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>

#include "ufr.h"

#define G_DL_MAX  16
uint8_t g_dl_count = 0;
void* g_dl_handles[G_DL_MAX];

extern uint8_t g_callback_array_count;
extern loop_callback g_callback_array[5];

// ============================================================================
//  Library Destructor
// ============================================================================

#include <signal.h>

extern volatile bool g_is_ok;

void ufr_linux_int_handler(int dummy) {
    printf("# CTRL-C Interruption\n");
    if ( g_is_ok ) {
        ufr_loop_set_end();
    } else {
        exit(1);
    }
}

__attribute__((constructor))
void ufr_linux_constructor() {
    printf("# Init UFR\n");
    signal(SIGINT, ufr_linux_int_handler);
}

__attribute__((destructor))
void ufr_linux_free_libraries() {
    for (uint8_t i=0; i<g_dl_count; i++) {
        if ( g_dl_handles[i] != NULL ) {
            dlclose(g_dl_handles[i]);
            g_dl_handles[i] = 0;
        }
    }
}

// ============================================================================
//  Functions
// ============================================================================

void* ufr_linux_load_library(const char* type, const char* name, const char* classname) {
    // dl_file: ufr_[gtw,enc,dcr]_name.so
    char dl_file[512];
    snprintf(dl_file, sizeof(dl_file), "libufr_%s_%s.so", type, name);

    // dl_funcname: ufr_gtw_mqtt_new, ufr_gtw_mqtt_new_topic
    char dl_funcname[1024];
    if ( classname[0] == '\0' ) {
        snprintf(dl_funcname, sizeof(dl_funcname), "ufr_%s_%s_new", type, name);
    } else {
        snprintf(dl_funcname, sizeof(dl_funcname), "ufr_%s_%s_new_%s", type, name, classname);
    }

    // open the dinamic library handle
    void* dl_handle = dlopen(dl_file, RTLD_LAZY);
    if ( dl_handle == NULL ) {
        // ufr_fatal(link, 1, dlerror());
        fprintf(stderr, "Error: %s\n", dlerror());
        exit(1);
    }

    // get the function pointer
    // ufr_info(link, "getting function %s", dl_funcname);
    dl_func_new_t dl_func_new = (dl_func_new_t) dlsym(dl_handle, dl_funcname);
    if ( dl_func_new == NULL ) {
        // ufr_fatal(link, 1, dlerror());
        fprintf(stderr, "Error: %s\n", dlerror());
        exit(1);
    }

    // check if handle is already in the array g_dl_handles 
    bool is_already_there = false;
    for (uint8_t i=0; i<g_dl_count; i++) {
        if ( g_dl_handles[i] == dl_handle ) {
            is_already_there = true;
            break;
        }
    }

    // put the handle pointer to the array g_dl_handles
    if ( is_already_there == false ) {
        g_dl_handles[ g_dl_count ] = dl_handle;
        g_dl_count += 1;
    }

    // success
    return (void*) dl_func_new;
}


int ufr_recv_2s(link_t* link0, link_t* link1, int time_ms) {
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
        i += 1;
    } while ( i < max );
    goto timeout;

second:
    do {
        usleep(time_us);
        if ( ufr_recv_async(link1) == UFR_OK ) {
            goto success;
        }
        i += 1;
    } while ( i < max );
    goto timeout;

timeout:
    return -1;

success:
    return UFR_OK;
}

static
uint64_t ufr_linux_timestamp_ms() {
    struct timespec now;
    timespec_get(&now, TIME_UTC);
    return ((uint64_t) now.tv_sec) * 1000 + ((uint64_t) now.tv_nsec) / 1000000;
}

bool ufr_linux_loop_rt(int32_t total_ms) {
    static int64_t last_ms = 0;

    // first execution
    if ( last_ms == 0 ) {
        last_ms = ufr_linux_timestamp_ms();
        return true;
    }

    // check if continue
    for (uint8_t i=0; i<g_callback_array_count; i++) {
        if ( g_callback_array[i]() != UFR_OK ) {
            return false;
        }
    }

    // sleep by 100ms
    int64_t current_ms = ufr_linux_timestamp_ms();
    int32_t diff_ms = current_ms - last_ms;
    int32_t sleep_ms = total_ms - diff_ms;
    if ( sleep_ms > 2 ) {
        // printf("dormiu %ld\n", (sleep_ms-1) * 1000 );
        usleep( (sleep_ms-1) * 1000 );
    }

    if ( sleep_ms < 0 ) {
        printf("real time error %d\n", sleep_ms);
    } else {
        // polling the time
        do {
            current_ms = ufr_linux_timestamp_ms();
            diff_ms = current_ms - last_ms;
        } while( diff_ms < total_ms );
        // printf("opa %ld\n", current_ms);
    }

    // end
    last_ms = current_ms;
    return g_is_ok;
}