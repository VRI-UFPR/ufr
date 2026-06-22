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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ufr.h>

typedef struct {
    int a;
} ll_encoder_t;

// ============================================================================
//  MsgPack Driver
// ============================================================================

static
int ufr_enc_cbor_init(link_t* link, const ufr_args_t* args) {
    ll_encoder_t* enc_obj = malloc( sizeof(ll_encoder_t) );
    if ( enc_obj == NULL ) {
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }
    
    link->enc_obj = enc_obj;
    return UFR_OK;
}

static
void ufr_enc_cbor_free(link_t* link) {
    if ( link->enc_obj != NULL ) {
        free(link->enc_obj);
        link->enc_obj = NULL;
    }
}

static
int ufr_enc_cbor_cmd_clear(link_t* link) {
    ll_encoder_t* enc_obj = link->enc_obj;
    
    return UFR_OK;
}

static
int ufr_enc_cbor_put_u32(link_t* link, const uint32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            
        }
    }
    return wrote;
}

static
int ufr_enc_cbor_put_i32(link_t* link, const int32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            
        }
    }
    return wrote;
}

static
int ufr_enc_cbor_put_f32(link_t* link, const float* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            
        }
    }
    return wrote;
}

static
int ufr_enc_cbor_put_u64(link_t* link, const uint64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            
        }
    }
    return wrote;
}

static
int ufr_enc_cbor_put_i64(link_t* link, const int64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            
        }
    }
    return wrote;
}

static
int ufr_enc_cbor_put_f64(link_t* link, const double* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            
        }
    }
    return wrote;
}


static
int ufr_enc_cbor_put_str(link_t* link, const char* val) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const size_t size = strlen(val);
        
    }
    return 0;
}

static
int ufr_enc_cbor_put_cmd(link_t* link, char cmd) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( cmd == '\n' || cmd == (char) EOF ) {
        
        
    } else {
        return ufr_error(link, 1, "Command %d not found", cmd);
    }

    return UFR_OK;
}

static
int ufr_enc_cbor_put_raw(link_t* link, const uint8_t* buffer, int size) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    return size;
}

static
int ufr_enc_cbor_put_bin(link_t* link, const char* mime, const char* buffer, int nbytes) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    // build the binary package (mime:\0data)
    const int len_mime = strlen(mime);
    const int len_divisor = 1;
    

    // ok
    return nbytes;
}


int ufr_enc_cbor_cmd_enter(link_t* link, size_t maxsize) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    
    return UFR_OK;
}


int ufr_enc_cbor_cmd_leave(link_t* link) {
    return UFR_OK;
}

int ufr_enc_cbor_cmd_eof(link_t* link) {
    ll_encoder_t* enc_obj = link->enc_obj;
    
    // ufr_write(link, data, size);
    
    return UFR_OK;
}

static
ufr_enc_api_t ufr_enc_cbor_api = {
    .init = ufr_enc_cbor_init,
    .free = ufr_enc_cbor_free,

    .put_u32 = ufr_enc_cbor_put_u32,
    .put_i32 = ufr_enc_cbor_put_i32,
    .put_f32 = ufr_enc_cbor_put_f32,

    .put_u64 = ufr_enc_cbor_put_u64,
    .put_i64 = ufr_enc_cbor_put_i64,
    .put_f64 = ufr_enc_cbor_put_f64,

    .put_str = ufr_enc_cbor_put_str,
    .put_raw = ufr_enc_cbor_put_raw,
    .put_bin = ufr_enc_cbor_put_bin,

    .cmd_enter = ufr_enc_cbor_cmd_enter,
    .cmd_leave = ufr_enc_cbor_cmd_leave,
    .cmd_next = NULL,
    .cmd_clear = ufr_enc_cbor_cmd_clear,
    .cmd_send = NULL,
    .cmd_eof = ufr_enc_cbor_cmd_eof
};

// ============================================================================
//  Public
// ============================================================================

int ufr_enc_cbor_new(link_t* link) {
    link->enc_api = &ufr_enc_cbor_api;
    return UFR_OK;
}

