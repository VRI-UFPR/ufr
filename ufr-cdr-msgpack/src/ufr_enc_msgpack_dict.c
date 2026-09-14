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

#define MAX_ITEMS     64

#define NULO          0

#define SCALAR_U8     1
#define SCALAR_U16    2
#define SCALAR_U32    3
#define SCALAR_U64    4
#define SCALAR_I8     5
#define SCALAR_I16    6
#define SCALAR_I32    7
#define SCALAR_I64    8
#define SCALAR_F32    9
#define SCALAR_F64    10
#define SCALAR_STR    11
#define SCALAR_BIN    12

#define ARRAY_U8      41
#define ARRAY_U16     42
#define ARRAY_U32     43
#define ARRAY_U64     44
#define ARRAY_I8      45
#define ARRAY_I16     46
#define ARRAY_I32     47
#define ARRAY_I64     48
#define ARRAY_F32     49
#define ARRAY_F64     50
#define ARRAY_STR     51
#define ARRAY_BIN     52



typedef struct {
    int8_t type;
    char name[67];
    int32_t nitems;

    union {
        uint8_t u8;
        uint16_t u16;
        uint32_t u32;
        uint64_t u64;
        int8_t i8;
        int16_t i16;
        int32_t i32;
        int64_t i64;
        float  f32;
        double f64;
        const void* ptr;
        const char* str;
    };
} ll_item_t;


typedef struct {
    msgpack_sbuffer sbuf;
    msgpack_packer pk;

    int32_t index;
    ll_item_t data[MAX_ITEMS];
} ll_encoder_t;

static int ufr_enc_dict_cmd_clear(link_t* link);

// ============================================================================
//  MsgPack Driver
// ============================================================================

void packer_init(ll_encoder_t* enc_obj) {
    msgpack_sbuffer_init(&enc_obj->sbuf);
    msgpack_packer_init(&enc_obj->pk, &enc_obj->sbuf, msgpack_sbuffer_write);
}

void packer_free(ll_encoder_t* enc_obj) {

}

void packer_encode(link_t* link) {
    ll_encoder_t* enc_obj = link->enc_obj;
    const int nitems = enc_obj->index;
    for (int i=0; i<nitems; i++) {

    }
}

void packer_clear(ll_encoder_t* enc_obj) {
    msgpack_sbuffer_clear(&enc_obj->sbuf);
}

// ============================================================================
//  Driver
// ============================================================================

static
int ufr_enc_dict_init(link_t* link, const ufr_args_t* args) {
    ll_encoder_t* enc_obj = malloc( sizeof(ll_encoder_t) );
    if ( enc_obj == NULL ) {
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }
    packer_init(enc_obj);
    ufr_enc_dict_cmd_clear(link);
    link->enc_obj = enc_obj;
    return UFR_OK;
}

static
void ufr_enc_dict_free(link_t* link) {
    if ( link->enc_obj != NULL ) {
        free(link->enc_obj);
        link->enc_obj = NULL;
    }
}

// ==== Put 8 bits ====

static
int ufr_enc_dict_put_u8(link_t* link, const uint8_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const int index = enc_obj->index;
        if (index >= MAX_ITEMS){return -1;}

        if ( nitems == 1 ) {
            enc_obj->data[index].type = SCALAR_U8;
            enc_obj->data[index].u8 = val[0];
            enc_obj->data[index].nitems = 1;
        } else {
            enc_obj->data[index].type = ARRAY_U8;
            enc_obj->data[index].nitems = nitems;
            enc_obj->data[index].ptr = (void*) val;
        }
        enc_obj->index = index + 1;
    }
    return wrote;
}

static
int ufr_enc_dict_put_i8(link_t* link, const int8_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const int index = enc_obj->index;
        if (index >= MAX_ITEMS){return -1;}

        if ( nitems == 1 ) {
            enc_obj->data[index].type = SCALAR_I8;
            enc_obj->data[index].i8 = val[0];
            enc_obj->data[index].nitems = 1;
        } else {
            enc_obj->data[index].type = ARRAY_I8;
            enc_obj->data[index].nitems = nitems;
            enc_obj->data[index].ptr = (void*) val;
        }
        enc_obj->index = index + 1;
    }
    return wrote;
}

// ==== Put 16 bits ====

static
int ufr_enc_dict_put_u16(link_t* link, const uint16_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const int index = enc_obj->index;
        if (index >= MAX_ITEMS ){return -1;}

        if ( nitems == 1 ) {
            enc_obj->data[index].type = SCALAR_U16;
            enc_obj->data[index].u16 = val[0];
            enc_obj->data[index].nitems = 1;
        } else {
            enc_obj->data[index].type = ARRAY_U16;
            enc_obj->data[index].nitems = nitems;
            enc_obj->data[index].ptr = (void*) val;
        }
        enc_obj->index = index + 1;
    }
    return wrote;
}

static
int ufr_enc_dict_put_i16(link_t* link, const int16_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const int index = enc_obj->index;
        if (index >= MAX_ITEMS ){return -1;}

        if ( nitems == 1 ) {
            enc_obj->data[index].type = SCALAR_I16;
            enc_obj->data[index].i16 = val[0];
            enc_obj->data[index].nitems = 1;
        } else {
            enc_obj->data[index].type = ARRAY_I16;
            enc_obj->data[index].nitems = nitems;
            enc_obj->data[index].ptr = (void*) val;
        }
        enc_obj->index = index + 1;
    }
    return wrote;
}

// ==== Put 32 bits ====

static
int ufr_enc_dict_put_u32(link_t* link, const uint32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const int index = enc_obj->index;
        if (index >= MAX_ITEMS ){return -1;}

        if ( nitems == 1 ) {
            enc_obj->data[index].type = SCALAR_U32;
            enc_obj->data[index].u32 = val[0];
            enc_obj->data[index].nitems = 1;
        } else {
            enc_obj->data[index].type = ARRAY_U32;
            enc_obj->data[index].nitems = nitems;
            enc_obj->data[index].ptr = (void*) val;
        }
        enc_obj->index = index + 1;
    }
    return wrote;
}

static
int ufr_enc_dict_put_i32(link_t* link, const int32_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const int index = enc_obj->index;
        if (index >= MAX_ITEMS ){return -1;}

        if ( nitems == 1 ) {
            enc_obj->data[index].type = SCALAR_I32;
            enc_obj->data[index].i32 = val[0];
            enc_obj->data[index].nitems = 1;
        } else {
            enc_obj->data[index].type = ARRAY_I32;
            enc_obj->data[index].nitems = nitems;
            enc_obj->data[index].ptr = (void*) val;
        }
        enc_obj->index = index + 1;
    }
    return wrote;
}

static
int ufr_enc_dict_put_f32(link_t* link, const float* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const int index = enc_obj->index;
        if (index >= MAX_ITEMS ){return -1;}

        if ( nitems == 1 ) {
            enc_obj->data[index].type = SCALAR_F32;
            enc_obj->data[index].f32 = val[0];
            enc_obj->data[index].nitems = 1;
        } else {
            enc_obj->data[index].type = ARRAY_F32;
            enc_obj->data[index].nitems = nitems;
            enc_obj->data[index].ptr = (void*) val;
        }
        enc_obj->index = index + 1;
    }
    return wrote;
}

// ==== Put 64 bits ====

static
int ufr_enc_dict_put_u64(link_t* link, const uint64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const int index = enc_obj->index;
        if (index >= MAX_ITEMS ){return -1;}

        if ( nitems == 1 ) {
            enc_obj->data[index].type = SCALAR_U64;
            enc_obj->data[index].u64 = val[0];
            enc_obj->data[index].nitems = 1;
        } else {
            enc_obj->data[index].type = ARRAY_U64;
            enc_obj->data[index].nitems = nitems;
            enc_obj->data[index].ptr = (void*) val;
        }
        enc_obj->index = index + 1;
    }
    return wrote;
}

static
int ufr_enc_dict_put_i64(link_t* link, const int64_t* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const int index = enc_obj->index;
        if (index >= MAX_ITEMS ){return -1;}

        if ( nitems == 1 ) {
            enc_obj->data[index].type = SCALAR_I64;
            enc_obj->data[index].i64 = val[0];
            enc_obj->data[index].nitems = 1;
        } else {
            enc_obj->data[index].type = ARRAY_I64;
            enc_obj->data[index].nitems = nitems;
            enc_obj->data[index].ptr = (void*) val;
        }
        enc_obj->index = index + 1;
    }
    return wrote;
}

static
int ufr_enc_dict_put_f64(link_t* link, const double* val, int nitems) {
    int wrote = 0;
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        const int index = enc_obj->index;
        if (index >= MAX_ITEMS ){return -1;}

        if ( nitems == 1 ) {
            enc_obj->data[index].type = SCALAR_F64;
            enc_obj->data[index].f64 = val[0];
            enc_obj->data[index].nitems = 1;
        } else {
            enc_obj->data[index].type = ARRAY_F64;
            enc_obj->data[index].nitems = nitems;
            enc_obj->data[index].ptr = (void*) val;
        }
        enc_obj->index = index + 1;
    }
    return wrote;
}

// ==== Put One ====

static
int ufr_enc_dict_put_one_str(link_t* link, const char* val) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj ) {
        // const size_t size = strlen(val);
        // msgpack_pack_str(&enc_obj->pk, size);
        // msgpack_pack_str_body(&enc_obj->pk, val, size);
        const int index = enc_obj->index;
        enc_obj->data[index].type = ARRAY_F64;
        enc_obj->data[index].nitems = 1;
        enc_obj->data[index].str = val;
        enc_obj->index = index + 1;
    }
    return 0;
}

static
int ufr_enc_dict_put_one_raw(link_t* link, const uint8_t* buffer, int size) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    // msgpack_pack_bin(&enc_obj->pk, size);
    // msgpack_pack_bin_body(&enc_obj->pk, buffer, size);
    return size;
}

static
int ufr_enc_dict_put_one_bin(link_t* link, const char* mime, const char* buffer, int nbytes) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    // build the binary package (mime:\0data)
    // const int len_mime = strlen(mime);
    // const int len_divisor = 1;

    /*
    msgpack_pack_bin(&enc_obj->pk, len_mime+len_divisor+nbytes);
    msgpack_pack_bin_body(&enc_obj->pk, mime, len_mime);
    msgpack_pack_bin_body(&enc_obj->pk, "\0", len_divisor);
    msgpack_pack_bin_body(&enc_obj->pk, buffer, nbytes);
    */

    // ok
    return nbytes;
}

// ==== Commands ====

static
int ufr_enc_dict_cmd_send(link_t* link) {
    ll_encoder_t* enc_obj = link->enc_obj;

    // Empacota os dados
    packer_encode(link);

    // Envia os dados
    const size_t size = enc_obj->sbuf.size;
    const char* data = enc_obj->sbuf.data;
    ufr_write(link, data, size);
    msgpack_sbuffer_clear(&enc_obj->sbuf);
    return UFR_OK;
}

static
int ufr_enc_dict_cmd_enter(link_t* link, size_t maxsize) {
    // ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    // msgpack_pack_array(&enc_obj->pk, maxsize);
    return UFR_OK;
}

static
int ufr_enc_dict_cmd_leave(link_t* link) {
    return UFR_OK;
}

static
int ufr_enc_dict_cmd_next(link_t* link) {
    ll_encoder_t* enc_obj = link->enc_obj;
    if ( enc_obj->index >= MAX_ITEMS ) {
        return -1;
    }
    enc_obj->index += 1;
    return UFR_OK;
}

static
int ufr_enc_dict_cmd_clear(link_t* link) {
    ll_encoder_t* enc_obj = link->enc_obj;
    enc_obj->index = 0;
    packer_clear(enc_obj);
    return UFR_OK;
}

static
int ufr_enc_dict_cmd_eof(link_t* link) {
    ll_encoder_t* enc_obj = link->enc_obj;
    const size_t size = enc_obj->sbuf.size;
    const char* data = enc_obj->sbuf.data;
    ufr_write(link, data, size);
    packer_clear(enc_obj);
    return UFR_OK;
}

static
int ufr_enc_dict_cmd_seek_str(link_t* link, const char* name) {
    ll_encoder_t* enc_obj = link->enc_obj;
    const int index = enc_obj->index;
    strncpy(enc_obj->data[index].name, name, 64);
    return UFR_OK;
}

// ==== API ====

static
ufr_enc_api_t ufr_enc_dict_api = {
    .init = ufr_enc_dict_init,
    .free = ufr_enc_dict_free,

    // 8 bits
    .put_u8 = ufr_enc_dict_put_u8,
    .put_i8 = ufr_enc_dict_put_i8,

    // 16 bits
    .put_u16 = ufr_enc_dict_put_u16,
    .put_i16 = ufr_enc_dict_put_i16,

    // 32 bits
    .put_u32 = ufr_enc_dict_put_u32,
    .put_i32 = ufr_enc_dict_put_i32,
    .put_f32 = ufr_enc_dict_put_f32,

    // 64 bits
    .put_u64 = ufr_enc_dict_put_u64,
    .put_i64 = ufr_enc_dict_put_i64,
    .put_f64 = ufr_enc_dict_put_f64,

    // One Variable
    .put_str = ufr_enc_dict_put_one_str,
    .put_raw = ufr_enc_dict_put_one_raw,
    .put_bin = ufr_enc_dict_put_one_bin,

    // Commands
    .cmd_enter = ufr_enc_dict_cmd_enter,
    .cmd_leave = ufr_enc_dict_cmd_leave,
    .cmd_next = ufr_enc_dict_cmd_next,
    .cmd_clear = ufr_enc_dict_cmd_clear,
    .cmd_send = ufr_enc_dict_cmd_send,
    .cmd_eof = ufr_enc_dict_cmd_eof,
    .cmd_seek_str = ufr_enc_dict_cmd_seek_str
};

// ============================================================================
//  Public
// ============================================================================

int ufr_enc_dict_new(link_t* link) {
    link->enc_api = &ufr_enc_dict_api;
    return UFR_OK;
}

