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

#include "ufr_dcr_msgpack.h"

// ============================================================================
//  MsgPack Array
// ============================================================================

static
int ufr_dcr_msgpack_array_init(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

static
void ufr_dcr_msgpack_array_free(link_t* link) {
}

static
int ufr_dcr_msgpack_array_cmd_next(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    decoder->l0_idx += 1;
    return UFR_OK;
}

static
int ufr_dcr_msgpack_array_recv_cb(link_t* link, char* pack_data, size_t pack_nbytes) {
    return UFR_OK;
}

static
char ufr_dcr_msgpack_array_get_type(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    const int type = decoder->object.type;
    if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
        return 'i';
    }

    if ( type == MSGPACK_OBJECT_FLOAT32 ) {
        return 'f';
    }

    if ( type == MSGPACK_OBJECT_FLOAT64 ) {
        return 'g';
    }

    if ( type == MSGPACK_OBJECT_ARRAY ) {
        return 'a';
    }

    if ( type == MSGPACK_OBJECT_STR ) {
        return 's';
    }

    if ( type == MSGPACK_OBJECT_BIN ) {
        return 'r';
    }

    return 0;
}

static
int ufr_dcr_msgpack_array_get_nbytes(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    const int type = decoder->object.type;
    switch (type) {
        case MSGPACK_OBJECT_POSITIVE_INTEGER:
            return sizeof(uint64_t);

        case MSGPACK_OBJECT_NEGATIVE_INTEGER:
            return sizeof(int64_t);

        case MSGPACK_OBJECT_FLOAT32:
            return sizeof(float);

        case MSGPACK_OBJECT_FLOAT64:
            return sizeof(double);
        
        case MSGPACK_OBJECT_ARRAY:
            return decoder->object.via.array.size;

        case MSGPACK_OBJECT_STR:
            return decoder->object.via.str.size;

        case MSGPACK_OBJECT_BIN:
            return decoder->object.via.bin.size;

        default:
            return 0;
    }
}

/*
static
int ufr_dcr_msgpack_array_get_nitems(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    const int type = decoder->object.type;
    switch (type) {
        case MSGPACK_OBJECT_POSITIVE_INTEGER:
            return 1;

        case MSGPACK_OBJECT_NEGATIVE_INTEGER:
            return 1;

        case MSGPACK_OBJECT_FLOAT32:
            return 1;

        case MSGPACK_OBJECT_FLOAT64:
            return 1;
        
        case MSGPACK_OBJECT_ARRAY:
            return decoder->object.via.array.size;

        case MSGPACK_OBJECT_STR:
            return decoder->object.via.str.size;

        case MSGPACK_OBJECT_BIN:
            return decoder->object.via.bin.size;

        default:
            return 0;
    }
}
*/

static
uint8_t* ufr_dcr_msgpack_array_get_rawptr(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder->object.type != MSGPACK_OBJECT_BIN ) {
        return NULL;
    }
    return (uint8_t*) decoder->object.via.bin.ptr;
}

static
int ufr_dcr_msgpack_array_get_raw(link_t* link, uint8_t* out_val, int maxlen) {
    // get Decoder
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    // get item
    size_t size = 0;
    if ( decoder->l0_idx < decoder->l0_array.size ) {
        const msgpack_object item = decoder->l0_array.ptr[ decoder->l0_idx ];
        const int type = item.type;
        if ( type == MSGPACK_OBJECT_STR ) {
            const char* ptr = item.via.str.ptr;
            size = item.via.str.size;
            // BUG: verificar se size eh maior que maxlen
            memcpy(out_val, ptr, size);
        } else if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
            size = (maxlen < sizeof(uint64_t)) ? maxlen : sizeof(uint64_t);
            memcpy(out_val, &item.via.u64, size);
        } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
            size = (maxlen < sizeof(int64_t)) ? maxlen : sizeof(uint64_t);
            memcpy(out_val, &item.via.i64, size);
        } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
            size = (maxlen < sizeof(float)) ? maxlen : sizeof(float);
            memcpy(out_val, &item.via.f64, size);
        } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
            size = (maxlen < sizeof(double)) ? maxlen : sizeof(double);
            memcpy(out_val, &item.via.f64, size);
        } else {
            return -1;
        }
    } else {
        return -1;
    }

    // Success
    ufr_dcr_msgpack_array_cmd_next(link);
    return size;
}

static
int ufr_dcr_msgpack_array_get_str(link_t* link, char* out_val, int maxlen) {
    // set "" as return default
    out_val[0] = '\0';

    // get Decoder
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 1;
    }

    if ( decoder->l0_idx < decoder->l0_array.size ) {
        const msgpack_object item = decoder->l0_array.ptr[ decoder->l0_idx ];
        const int type = item.type;
        if ( type == MSGPACK_OBJECT_STR ) {
            const char* ptr = item.via.str.ptr;
            const size_t size = item.via.str.size;
            // BUG: verificar se size eh maior que maxlen
            strncpy(out_val, ptr, size);
            out_val[size] = '\0';
        } else if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
            const size_t copied = snprintf(out_val, maxlen, "%lu", item.via.u64);
            out_val[copied] = '\0';
        } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
            const size_t copied = snprintf(out_val, maxlen, "%ld", item.via.i64);
            out_val[copied] = '\0';
        } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
            const size_t copied = snprintf(out_val, maxlen, "%g", item.via.f64);
            out_val[copied] = '\0';
        } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
            const size_t copied = snprintf(out_val, maxlen, "%g", item.via.f64);
            out_val[copied] = '\0';
        } else {
            return 1;
        }
    } else {
        return -1;
    }

    // Success
    ufr_dcr_msgpack_array_cmd_next(link);
    return UFR_OK;
}

static
int ufr_dcr_msgpack_array_get_u32(link_t* link, uint32_t out_val[], int max_nitems) {
    // get Decoder
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    // Read the data
    const int n_items = (max_nitems < decoder->l0_array.size) ? max_nitems : decoder->l0_array.size;
    int read_i = 0;
    for (; read_i<n_items; read_i++) {
        if ( decoder->l0_idx < decoder->l0_array.size ) {
            const msgpack_object item = decoder->l0_array.ptr[ decoder->l0_idx ];
            const int type = item.type;
            if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
                out_val[read_i] = (uint32_t) item.via.u64;
            } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
                out_val[read_i] = (uint32_t) item.via.i64;
            } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
                out_val[read_i] = (uint32_t) item.via.f64;
            } else {
                break;
            }
        } else {
            break;
        }

        ufr_dcr_msgpack_array_cmd_next(link);
    }

    // success
    return read_i;
}

static
int ufr_dcr_msgpack_array_get_i32(link_t* link, int32_t out_val[], int max_nitems) {
    // get Decoder
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    // Read the data
    const int n_items = (max_nitems < decoder->l0_array.size) ? max_nitems : decoder->l0_array.size;
    int read_i = 0;
    for (; read_i<n_items; read_i++) {
        if ( decoder->l0_idx < decoder->l0_array.size ) {
            const msgpack_object item = decoder->l0_array.ptr[ decoder->l0_idx ];
            const int type = item.type;
            if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
                out_val[read_i] = (int32_t) item.via.u64;
            } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
                out_val[read_i] = (int32_t) item.via.i64;
            } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
                out_val[read_i] = (int32_t) item.via.f64;
            } else {
                break;
            }
        } else {
            break;
        }

        ufr_dcr_msgpack_array_cmd_next(link);
    }

    // success
    return read_i;
}

static
int ufr_dcr_msgpack_array_get_f32(link_t* link, float out_val[], int max_nitems) {
    // get Decoder
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    // Read the data
    const int n_items = (max_nitems < decoder->l0_array.size) ? max_nitems : decoder->l0_array.size;
    int read_i = 0;
    for (; read_i<n_items; read_i++) {
        if ( decoder->l0_idx < decoder->l0_array.size ) {
            const msgpack_object item = decoder->l0_array.ptr[ decoder->l0_idx ];
            const int type = item.type;
            if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
                out_val[read_i] = (float) item.via.u64;
            } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
                out_val[read_i] = (float) item.via.i64;
            } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
                out_val[read_i] = (float) item.via.f64;
            } else {
                break;
            }
        } else {
            break;
        }

        ufr_dcr_msgpack_array_cmd_next(link);
    }

    // success
    return read_i;
}

int ufr_dcr_msgpack_array_cmd_enter(link_t* link) {
    return -1;
}

int ufr_dcr_msgpack_array_cmd_leave(link_t* link) {
    link->dcr_api = link->dcr_api_s0;
    return UFR_OK;
}

// ============================================================================
//  MsgPack Array API
// ============================================================================

ufr_dcr_api_t ufr_dcr_msgpack_array_api = {
    .init = ufr_dcr_msgpack_array_init,
    .free = ufr_dcr_msgpack_array_free,

    .recv_cb = ufr_dcr_msgpack_array_recv_cb,
    .recv_async_cb = ufr_dcr_msgpack_array_recv_cb,

    // 32 bits
    .get_u32 = ufr_dcr_msgpack_array_get_u32,
    .get_i32 = ufr_dcr_msgpack_array_get_i32,
    .get_f32 = ufr_dcr_msgpack_array_get_f32,

    // 64 bits
    .get_u64 = NULL,
    .get_i64 = NULL,
    .get_f64 = NULL,

    // 8 bits
    .get_raw = ufr_dcr_msgpack_array_get_raw,
    .get_str = ufr_dcr_msgpack_array_get_str,
    .get_bin = NULL,
    .get_ptr = NULL,

    // enter/leave
    .cmd_enter = ufr_dcr_msgpack_array_cmd_enter,
    .cmd_leave = ufr_dcr_msgpack_array_cmd_leave,
    .cmd_next = ufr_dcr_msgpack_array_cmd_next,

    .meta_get = NULL,
    
    .meta_item_type = NULL,
    .meta_item_mime = NULL,
    .meta_item_nbytes = NULL,
    .meta_item_nitems = NULL,

    .meta_pack_mime = NULL,
    .meta_pack_nbytes = NULL,
    .meta_pack_nitems = NULL,
};
