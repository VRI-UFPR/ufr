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
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ufr.h>

typedef struct {
    uint32_t idx;
    uint32_t size;
    uint32_t maxsize;
    uint8_t* data;
} buffer_t;

// ============================================================================
//  File
// ============================================================================

static
int ufr_gtw_memory_type(const link_t* link) {
	return 0;
}

static
int ufr_gtw_memory_state(const link_t* link){
	return 0;
}

static
size_t ufr_gtw_memory_size(const link_t* link, int type){
	return 0;
}

static
int ufr_gtw_memory_boot(link_t* link, const ufr_args_t* args) {
    buffer_t* buffer = malloc( sizeof(buffer_t) );
    if ( buffer == NULL ) {
        ufr_fatal(link, 1, "Memory fault");
    }

    uint8_t* buffer_ptr = ufr_args_getp(args, "@buffer", NULL);
    uint32_t maxsize = ufr_args_getu(args, "@max", 1024);

    printf("%p\n", buffer_ptr);

    buffer->idx = 0;
    buffer->size = 0;
    buffer->maxsize = maxsize;
    buffer->data = buffer_ptr;
    link->gtw_obj = buffer;

	return 0;
}

static
int ufr_gtw_memory_start(link_t* link, int type, const ufr_args_t* args) {
	return 0;
}

static
void ufr_gtw_memory_stop(link_t* link, int type) {
    // Free Gateway Shared Object
    buffer_t* buffer = link->gtw_obj;
    if ( buffer != NULL ) {
        free(buffer);
        link->gtw_obj = NULL;
    }
}

static
int ufr_gtw_memory_copy(link_t* link, link_t* out) {
    out->gtw_shr = link->gtw_shr;
	return 0;
}

static
size_t ufr_gtw_memory_read(link_t* link, char* out_buffer, size_t length) {
	// buffer_t* buffer = (buffer_t*) link->gtw_obj;
    return 0;
}

static
size_t ufr_gtw_memory_write(link_t* link, const char* buffer, size_t length) {
    buffer_t* obj = (buffer_t*) link->gtw_obj;
    if ( obj == NULL || obj->data == NULL ) {
        return 0;
    }
    
    memcpy(obj->data, buffer, length);
    obj->data[length] = '\0';
    return length;
}

static
int ufr_gtw_memory_recv(link_t* link) {
    return 0;
}

static
ufr_gtw_api_t ufr_gtw_memory = {
    .name = "memory",
	.type = ufr_gtw_memory_type,
	.state = ufr_gtw_memory_state,
	.size = ufr_gtw_memory_size,
	.boot = ufr_gtw_memory_boot,
	.start = ufr_gtw_memory_start,
	.stop = ufr_gtw_memory_stop,
	.copy = ufr_gtw_memory_copy,
	.read = ufr_gtw_memory_read,
	.write = ufr_gtw_memory_write,
    .recv = ufr_gtw_memory_recv
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_memory_new(link_t* link, int type) {
    ufr_link_init(link, &ufr_gtw_memory);
    return UFR_OK;
}