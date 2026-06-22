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

// ============================================================================
//  MsgPack Root
// ============================================================================

static
int ufr_dcr_msgpack_init(link_t* link, const ufr_args_t* args) {
    ll_decoder_t* dcr_obj = malloc( sizeof(ll_decoder_t) );
    if ( dcr_obj == NULL ) {
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }

    link->dcr_obj = dcr_obj;
    return UFR_OK;
}

static
void ufr_dcr_msgpack_free(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder != NULL ) {

        free(decoder);
        link->dcr_obj = NULL;
    }
}

static
int ufr_dcr_msgpack_next(link_t* link) {
    // parse the next object in the message
    ll_decoder_t* decoder = link->dcr_obj;
    size_t current = decoder->cursor;

    msgpack_unpack_return ret = msgpack_unpack_next(&decoder->result, (const char *) decoder->pack_data, decoder->pack_nbytes, &current);

    // error
    if ( ret != MSGPACK_UNPACK_SUCCESS ) {
        // ufr_info(&link, "Error in the unpacking the message %ld\n", decoder->pack_nbytes);
        decoder->object.type = MSGPACK_OBJECT_NIL;
        decoder->object.via.u64 = 0;
        return -1;
    }

    // update decoder object
    decoder->cursor = current;
    decoder->object = decoder->result.data;
    return UFR_OK;
}

static
int ufr_dcr_msgpack_recv_cb(link_t* link, char* pack_data, size_t pack_nbytes) {
    ll_decoder_t* decoder = link->dcr_obj;
    decoder->pack_data = (uint8_t*) pack_data;
    decoder->pack_nbytes = pack_nbytes;
    decoder->cursor = 0;
    decoder->is_pack_scalar = false;
    decoder->pack_nitems = -1;
    const int code = ufr_dcr_msgpack_next(link);
    if ( code == UFR_OK ) {
        if ( decoder->cursor >= decoder->pack_nbytes ) {
            decoder->is_pack_scalar = true;
            decoder->pack_nitems = 1;
        }
    }
    return code;
}


static
int ufr_dcr_msgpack_get_raw(link_t* link, uint8_t* out_val, int maxlen) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    size_t size = 0;
    const int type = decoder->object.type;
    if ( type == MSGPACK_OBJECT_BIN ) {
        const size_t object_size = decoder->object.via.bin.size;
        size = (maxlen < object_size) ? maxlen : object_size;
        memcpy(out_val, decoder->object.via.bin.ptr, size);
    } else if ( type == MSGPACK_OBJECT_STR ) {
        const char* ptr = decoder->object.via.str.ptr;
        size = ( maxlen < decoder->object.via.str.size ) ? maxlen : decoder->object.via.str.size;
        memcpy(out_val, ptr, size);
    } else if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
        size = (maxlen < sizeof(uint64_t)) ? maxlen : sizeof(uint64_t);
        memcpy(out_val, &decoder->object.via.u64, size);
    } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        size = (maxlen < sizeof(int64_t)) ? maxlen : sizeof(uint64_t);
        memcpy(out_val, &decoder->object.via.i64, size);
    } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
        size = (maxlen < sizeof(float)) ? maxlen : sizeof(float);
        memcpy(out_val, &decoder->object.via.f64, size);
    } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
        size = (maxlen < sizeof(double)) ? maxlen : sizeof(double);
        memcpy(out_val, &decoder->object.via.f64, size);
    } else {
        return 0;
    }

    // Success
    ufr_dcr_msgpack_next(link);
    return size;
}


static
int ufr_dcr_msgpack_get_bin(link_t* link, char** out_mime, char** out_data, int* out_nbytes) {
    // Get the decoder object
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    // Get the Binary Object
    const int type = decoder->object.type;
    if ( type == MSGPACK_OBJECT_BIN ) {
        const size_t object_size = decoder->object.via.bin.size;
        const char* mime = decoder->object.via.bin.ptr;
        const int mime_len = strlen(mime);
        
        // Set the output
        *out_mime = (char*) mime;
        *out_data = (char*) &decoder->object.via.bin.ptr[mime_len+1];  // +1: jump the \0 after the mime
        *out_nbytes = object_size - mime_len;
    } else {
        return -1;
    }

    // Success
    ufr_dcr_msgpack_next(link);
    return UFR_OK;
}


static
int ufr_dcr_msgpack_get_str(link_t* link, char* out_val, int maxlen) {
    out_val[0] = '\0';
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 1;
    }

    const int type = decoder->object.type;
    if ( type == MSGPACK_OBJECT_STR ) {
        const char* ptr = decoder->object.via.str.ptr;
        const size_t size = (decoder->object.via.str.size >= maxlen) ? maxlen-1 : decoder->object.via.str.size;
        strncpy(out_val, ptr, size);
        out_val[size] = '\0';
    } else if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
        const size_t copied = snprintf(out_val, maxlen, "%lu", decoder->object.via.u64);
        out_val[copied] = '\0';
    } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        const size_t copied = snprintf(out_val, maxlen, "%ld", decoder->object.via.i64);
        out_val[copied] = '\0';
    } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
        const size_t copied = snprintf(out_val, maxlen, "%g", decoder->object.via.f64);
        out_val[copied] = '\0';
    } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
        const size_t copied = snprintf(out_val, maxlen, "%g", decoder->object.via.f64);
        out_val[copied] = '\0';
    } else {
        return 1;
    }

    // Success
    ufr_dcr_msgpack_next(link);
    return UFR_OK;
}

static
int ufr_dcr_msgpack_get_u32(link_t* link, uint32_t out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    int wrote = 0;
    for (; wrote<max_nitems; wrote++) {
        // return the value
        const int type = decoder->object.type;
        if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
            out_val[wrote] = (uint32_t) decoder->object.via.u64;
        } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
            out_val[wrote] = (uint32_t) decoder->object.via.i64;
        } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
            out_val[wrote] = (uint32_t) decoder->object.via.f64;
        } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
            out_val[wrote] = (uint32_t) decoder->object.via.f64;
        } else {
            
        }

        // success
        if ( ufr_dcr_msgpack_next(link) != UFR_OK ) {
            wrote += 1;
            break;
        }
    }

    return wrote;
}

static
int ufr_dcr_msgpack_get_i32(link_t* link, int32_t out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    int wrote = 0;
    for (; wrote<max_nitems; wrote++) {
        // return the value
        const int type = decoder->object.type;
        if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
            out_val[wrote] = (int32_t) decoder->object.via.u64;
        } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
            out_val[wrote] = (int32_t) decoder->object.via.i64;
        } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
            out_val[wrote] = (int32_t) decoder->object.via.f64;
        } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
            out_val[wrote] = (int32_t) decoder->object.via.f64;
        } else {
            
        }

        // success
        if ( ufr_dcr_msgpack_next(link) != UFR_OK ) {
            wrote += 1;
            break;
        }
    }

    return wrote;
}

static
int ufr_dcr_msgpack_get_f32(link_t* link, float out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    int wrote = 0;
    for (; wrote<max_nitems; wrote++) {
        // return the value
        const int type = decoder->object.type;
        if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
            out_val[wrote] = (float) decoder->object.via.u64;
        } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
            out_val[wrote] = (float) decoder->object.via.i64;
        } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
            out_val[wrote] = (float) decoder->object.via.f64;
        } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
            out_val[wrote] = (float) decoder->object.via.f64;
        } else {
            // printf("error %d\n", type);
        }

        // success
        if ( ufr_dcr_msgpack_next(link) != UFR_OK ) {
            wrote += 1;
            break;
        }
    }

    return wrote;
}

static
int ufr_dcr_msgpack_get_u64(link_t* link, uint64_t out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    int wrote = 0;
    for (; wrote<max_nitems; wrote++) {
        // return the value
        const int type = decoder->object.type;
        if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
            out_val[wrote] = (uint64_t) decoder->object.via.u64;
        } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
            out_val[wrote] = (uint64_t) decoder->object.via.i64;
        } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
            out_val[wrote] = (uint64_t) decoder->object.via.f64;
        } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
            out_val[wrote] = (uint64_t) decoder->object.via.f64;
        } else {
            
        }

        // success
        if ( ufr_dcr_msgpack_next(link) != UFR_OK ) {
            break;
        }
    }

    return wrote;
}

static
int ufr_dcr_msgpack_get_i64(link_t* link, int64_t out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    int wrote = 0;
    for (; wrote<max_nitems; wrote++) {
        // return the value
        const int type = decoder->object.type;
        if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
            out_val[wrote] = (int64_t) decoder->object.via.u64;
        } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
            out_val[wrote] = (int64_t) decoder->object.via.i64;
        } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
            out_val[wrote] = (int64_t) decoder->object.via.f64;
        } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
            out_val[wrote] = (int64_t) decoder->object.via.f64;
        } else {
            
        }

        // success
        if ( ufr_dcr_msgpack_next(link) != UFR_OK ) {
            break;
        }
    }

    return wrote;
}

static
int ufr_dcr_msgpack_get_f64(link_t* link, double out_val[], int max_nitems) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    int wrote = 0;
    for (; wrote<max_nitems; wrote++) {
        // return the value
        const int type = decoder->object.type;
        if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
            out_val[wrote] = (double) decoder->object.via.u64;
        } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
            out_val[wrote] = (double) decoder->object.via.i64;
        } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
            out_val[wrote] = (double) decoder->object.via.f64;
        } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
            out_val[wrote] = (double) decoder->object.via.f64;
        } else {
            
        }

        // success
        if ( ufr_dcr_msgpack_next(link) != UFR_OK ) {
            break;
        }
    }

    return wrote;
}


void* ufr_dcr_msgpack_get_ptr(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    const int type = decoder->object.type;
    if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
        return (void*) &decoder->object.via.u64;
    }
    if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        return (void*) &decoder->object.via.i64;
    }
    if ( type == MSGPACK_OBJECT_FLOAT32 ) {
        return (void*) &decoder->object.via.f64;
    }
    if ( type == MSGPACK_OBJECT_FLOAT64 ) {
        return (void*) &decoder->object.via.f64;
    }
    if ( type == MSGPACK_OBJECT_STR ) {
        return (void*) decoder->object.via.str.ptr;
    }
    if ( type == MSGPACK_OBJECT_BIN ) {
        return (void*) decoder->object.via.bin.ptr;
    }
    if ( type == MSGPACK_OBJECT_ARRAY ) {
        return NULL;
    }
    return NULL;
}


char ufr_dcr_msgpack_meta_item_type(link_t* link) {
    return '\0';
}

const char* ufr_dcr_msgpack_meta_item_mime(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;

    const int type = decoder->object.type;
    if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
        return "number/u32";
    } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        return "number/i32";
    } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
        return "number/f32";
    } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
        return "number/f64";
    } else if ( type == MSGPACK_OBJECT_STR ) {
        return "text/plain";
    } else if ( type == MSGPACK_OBJECT_BIN ) {
        return decoder->object.via.bin.ptr;
    } else if ( type == MSGPACK_OBJECT_ARRAY ) {
        return "list";    
    }

    // error
    ufr_warn(link, "Variable type (%d) is unknown for the decoder", type);
    return "error";
}

int ufr_dcr_msgpack_meta_item_nbytes(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    
    const int type = decoder->object.type;
    if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
        return 4;
    } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        return 4;
    } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
        return 4;
    } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
        return 8;
    } else if ( type == MSGPACK_OBJECT_STR ) {
        return (int32_t) decoder->object.via.str.size;
    } else if ( type == MSGPACK_OBJECT_BIN ) {
        const size_t mime_len = strlen( (const char*)decoder->object.via.bin.ptr ) + 1; // +1: count the '\0' too
        return (int32_t) ( decoder->object.via.bin.size - mime_len );
    }

    if ( decoder->cursor >= decoder->pack_nbytes ) {
        printf("EOF\n");
    }

    // error
    ufr_warn(link, "Variable type (%d) is unknown for the decoder", type);
    return 0;
}

int ufr_dcr_msgpack_meta_item_nitems(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    const int type = decoder->object.type;
    if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
        return 1;
    } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
        return 1;
    } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
        return 1;
    } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
        return 1;
    } else if ( type == MSGPACK_OBJECT_STR ) {
        return (int32_t) decoder->object.via.str.size;
    } else if ( type == MSGPACK_OBJECT_BIN ) {
        const size_t mime_len = strlen( (const char*)decoder->object.via.bin.ptr ) + 1; // +1: count the '\0' too
        return (int32_t) decoder->object.via.bin.size - mime_len;
    } else if ( type == MSGPACK_OBJECT_ARRAY ) {
        return (int32_t) decoder->object.via.array.size;
    }

    // error
    ufr_warn(link, "Variable type (%d) is unknown for the decoder", type);
    return 0;
}



const char* ufr_dcr_msgpack_meta_pack_mime(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder->is_pack_scalar ) {
        const int type = decoder->object.type;
        if ( type == MSGPACK_OBJECT_POSITIVE_INTEGER ) {
            return "number/u32";
        } else if ( type == MSGPACK_OBJECT_NEGATIVE_INTEGER ) {
            return "number/i32";
        } else if ( type == MSGPACK_OBJECT_FLOAT32 ) {
            return "number/f32";
        } else if ( type == MSGPACK_OBJECT_FLOAT64 ) {
            return "number/f64";
        } else if ( type == MSGPACK_OBJECT_STR ) {
            return "text/plain";
        } else if ( type == MSGPACK_OBJECT_BIN ) {
            return decoder->object.via.bin.ptr;
        } else if ( type == MSGPACK_OBJECT_ARRAY ) {
            return "list";    
        }
    }
    return "list";
}

int ufr_dcr_msgpack_meta_pack_nbytes(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    return decoder->pack_nbytes;
}

int ufr_dcr_msgpack_meta_pack_nitems(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder == NULL ) {
        return 0;
    }

    if ( decoder->pack_nitems < 0 ) {
        int pack_nitems = 1;             // =1 -> first item already parsed
        size_t cursor = decoder->cursor;
        msgpack_unpacked result;
        msgpack_unpacked_init(&result);
        while (1) {
            const msgpack_unpack_return ret = msgpack_unpack_next(&result, (const char *) decoder->pack_data, decoder->pack_nbytes, &cursor);
            if ( ret != MSGPACK_UNPACK_SUCCESS ) {
                break;
            }
            pack_nitems += 1;
        }
        msgpack_unpacked_destroy(&result);
        decoder->pack_nitems = pack_nitems;
    }

    return decoder->pack_nitems;
}


int ufr_dcr_msgpack_cmd_enter(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    if ( decoder->object.type != MSGPACK_OBJECT_ARRAY ) {
        return -1;
    }

    decoder->l0_array = decoder->object.via.array;
    decoder->l0_idx = 0;
    link->dcr_api_s0 = link->dcr_api;
    link->dcr_api = &ufr_dcr_msgpack_array_api;
    return UFR_OK;
}

int ufr_dcr_msgpack_cmd_leave(link_t* link) {
    return -1;
}

static
ufr_dcr_api_t ufr_dcr_msgpack_api = {
    // Init/Free
    .init = ufr_dcr_msgpack_init,
    .free = ufr_dcr_msgpack_free,

    // recv
    .recv_cb = ufr_dcr_msgpack_recv_cb,
    .recv_async_cb = ufr_dcr_msgpack_recv_cb,

    // 32 bits
    .get_u32 = ufr_dcr_msgpack_get_u32,
    .get_i32 = ufr_dcr_msgpack_get_i32,
    .get_f32 = ufr_dcr_msgpack_get_f32,

    // 64 bits
    .get_u64 = ufr_dcr_msgpack_get_u64,
    .get_i64 = ufr_dcr_msgpack_get_i64,
    .get_f64 = ufr_dcr_msgpack_get_f64,

    // 8 bits
    .get_raw = ufr_dcr_msgpack_get_raw,
    .get_str = ufr_dcr_msgpack_get_str,
    .get_bin = ufr_dcr_msgpack_get_bin,
    .get_ptr = ufr_dcr_msgpack_get_ptr,

    // enter/leave
    .cmd_enter = ufr_dcr_msgpack_cmd_enter,
    .cmd_leave = ufr_dcr_msgpack_cmd_leave,
    .cmd_next = ufr_dcr_msgpack_next,

    // remove
    .meta_get = NULL,
    
    // Metadata for Item
    .meta_item_type = ufr_dcr_msgpack_meta_item_type,
    .meta_item_mime = ufr_dcr_msgpack_meta_item_mime,
    .meta_item_nbytes = ufr_dcr_msgpack_meta_item_nbytes,
    .meta_item_nitems = ufr_dcr_msgpack_meta_item_nitems,

    // Metadata for Package
    .meta_pack_mime = ufr_dcr_msgpack_meta_pack_mime,
    .meta_pack_nbytes = ufr_dcr_msgpack_meta_pack_nbytes,
    .meta_pack_nitems = ufr_dcr_msgpack_meta_pack_nitems,
};

// ============================================================================
//  Public
// ============================================================================

int ufr_dcr_msgpack_new(link_t* link) {
    link->dcr_api = &ufr_dcr_msgpack_api;
    return UFR_OK;
}
