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
#include <msgpack.h>
#include <ufr.h>

typedef struct {
    msgpack_sbuffer sbuf;
    msgpack_packer pk;
} ll_encoder_t;

// ============================================================================
//  MsgPack Driver
// ============================================================================

static
int ufr_enc_msgpack_init(link_t* link, const ufr_args_t* args) {
    ll_encoder_t* enc_obj = malloc( sizeof(ll_encoder_t) );
    if ( enc_obj == NULL ) {
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }
    msgpack_sbuffer_init(&enc_obj->sbuf);
    msgpack_packer_init(&enc_obj->pk, &enc_obj->sbuf, msgpack_sbuffer_write);
    link->enc_obj = enc_obj;
    return UFR_OK;
}

static
void ufr_enc_msgpack_free(link_t* link) {
    if ( link->enc_obj != NULL ) {
        free(link->enc_obj);
        link->enc_obj = NULL;
    }
}

// ==== Put 8 bits ====

static
int ufr_enc_msgpack_put_u8(link_t* link, const uint8_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_uint32(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

static
int ufr_enc_msgpack_put_i8(link_t* link, const int8_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_int32(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

// ==== Put 16 bits ====

static
int ufr_enc_msgpack_put_u16(link_t* link, const uint16_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_uint32(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

static
int ufr_enc_msgpack_put_i16(link_t* link, const int16_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_int32(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

// ==== Put 32 bits ====

static
int ufr_enc_msgpack_put_u32(link_t* link, const uint32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_uint32(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

static
int ufr_enc_msgpack_put_i32(link_t* link, const int32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_int32(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

static
int ufr_enc_msgpack_put_f32(link_t* link, const float* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_float(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

// ==== Put 64 bits ====

static
int ufr_enc_msgpack_put_u64(link_t* link, const uint64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_uint64(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

static
int ufr_enc_msgpack_put_i64(link_t* link, const int64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_int64(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

static
int ufr_enc_msgpack_put_f64(link_t* link, const double* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        for (; wrote<nitems; wrote++) {
            msgpack_pack_double(&enc_obj->pk, val[wrote]);
        }
    }
    return wrote;
}

// ==== Put One ====

static
int ufr_enc_msgpack_put_one_str(link_t* link, const char* val) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const size_t size = strlen(val);
        msgpack_pack_str(&enc_obj->pk, size);
        msgpack_pack_str_body(&enc_obj->pk, val, size);
    }
    return 0;
}

static
int ufr_enc_msgpack_put_one_raw(link_t* link, const uint8_t* buffer, int size) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    msgpack_pack_bin(&enc_obj->pk, size);
    msgpack_pack_bin_body(&enc_obj->pk, buffer, size);
    return size;
}

static
int ufr_enc_msgpack_put_one_bin(link_t* link, const char* mime, const char* buffer, int nbytes) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    // build the binary package (mime:\0data)
    const int len_mime = strlen(mime);
    const int len_divisor = 1;
    msgpack_pack_bin(&enc_obj->pk, len_mime+len_divisor+nbytes);
    msgpack_pack_bin_body(&enc_obj->pk, mime, len_mime);
    msgpack_pack_bin_body(&enc_obj->pk, "\0", len_divisor);
    msgpack_pack_bin_body(&enc_obj->pk, buffer, nbytes);

    // ok
    return nbytes;
}

// ==== Commands ====

static
int ufr_enc_msgpack_cmd_send(link_t* link) {
    ll_encoder_t* enc_obj = link->enc_obj;
    const size_t size = enc_obj->sbuf.size;
    const char* data = enc_obj->sbuf.data;
    ufr_write(link, data, size);
    msgpack_sbuffer_clear(&enc_obj->sbuf);
    return UFR_OK;
}

static
int ufr_enc_msgpack_cmd_enter(link_t* link, size_t maxsize) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    msgpack_pack_array(&enc_obj->pk, maxsize);
    return UFR_OK;
}

static
int ufr_enc_msgpack_cmd_leave(link_t* link) {
    return UFR_OK;
}

static
int ufr_enc_msgpack_cmd_next(link_t* link) {
    // ll_encoder_t* enc_obj = link->enc_obj;
    return UFR_OK;
}

static
int ufr_enc_msgpack_cmd_clear(link_t* link) {
    ll_encoder_t* enc_obj = link->enc_obj;
    msgpack_sbuffer_clear(&enc_obj->sbuf);
    return UFR_OK;
}

static
int ufr_enc_msgpack_cmd_eof(link_t* link) {
    ll_encoder_t* enc_obj = link->enc_obj;
    const size_t size = enc_obj->sbuf.size;
    const char* data = enc_obj->sbuf.data;
    ufr_write(link, data, size);
    msgpack_sbuffer_clear(&enc_obj->sbuf);
    return UFR_OK;
}

static
int ufr_enc_msgpack_cmd_seek_str(link_t* link, const char* name) {
    return UFR_OK;
}

// ==== API ====

static
ufr_enc_api_t ufr_enc_msgpack_api = {
    .init = ufr_enc_msgpack_init,
    .free = ufr_enc_msgpack_free,

    // 8 bits
    .put_u8 = ufr_enc_msgpack_put_u8,
    .put_i8 = ufr_enc_msgpack_put_i8,

    // 16 bits
    .put_u16 = ufr_enc_msgpack_put_u16,
    .put_i16 = ufr_enc_msgpack_put_i16,

    // 32 bits
    .put_u32 = ufr_enc_msgpack_put_u32,
    .put_i32 = ufr_enc_msgpack_put_i32,
    .put_f32 = ufr_enc_msgpack_put_f32,

    // 64 bits
    .put_u64 = ufr_enc_msgpack_put_u64,
    .put_i64 = ufr_enc_msgpack_put_i64,
    .put_f64 = ufr_enc_msgpack_put_f64,

    // One Variable
    .put_str = ufr_enc_msgpack_put_one_str,
    .put_raw = ufr_enc_msgpack_put_one_raw,
    .put_bin = ufr_enc_msgpack_put_one_bin,

    // Commands
    .cmd_enter = ufr_enc_msgpack_cmd_enter,
    .cmd_leave = ufr_enc_msgpack_cmd_leave,
    .cmd_next = ufr_enc_msgpack_cmd_next,
    .cmd_clear = ufr_enc_msgpack_cmd_clear,
    .cmd_send = ufr_enc_msgpack_cmd_send,
    .cmd_eof = ufr_enc_msgpack_cmd_eof,
    .cmd_seek_str = ufr_enc_msgpack_cmd_seek_str
};

// ============================================================================
//  Public
// ============================================================================

int ufr_enc_msgpack_new(link_t* link) {
    link->enc_api = &ufr_enc_msgpack_api;
    return UFR_OK;
}

