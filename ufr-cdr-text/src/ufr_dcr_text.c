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

#define TOKEN_SIZE 1024

typedef struct {
    char const* msg_ptr;
    uint32_t msg_size;
    uint32_t msg_index;
    char sep; 
} ll_decoder_t;

// ============================================================================
//  Private Functions
// ============================================================================

int lex_next_token(ll_decoder_t* decoder, char* out_token) {
    const char sep = decoder->sep;
    const char* msg = decoder->msg_ptr;
    const uint32_t size = decoder->msg_size;
    uint32_t cursor = decoder->msg_index;
    uint32_t i_token = 0;    

    while( cursor < size ) {
        const char c = msg[cursor];
        if ( c == '\n' || c == '\0' ) {
            break;
        } else {
            if ( i_token < TOKEN_SIZE-1 ) {
                out_token[i_token] = c;
                i_token += 1;
            }
        }

        cursor += 1;
    }

    out_token[i_token] = '\0';
    decoder->msg_index = cursor;
    return ( i_token > 0 ) ? UFR_OK : -1;
}

// ============================================================================
//  CSV Decoder
// ============================================================================

static
int ufr_dcr_text_boot(link_t* link, const ufr_args_t* args) {
    // allocate the decoder object
    ll_decoder_t* decoder = malloc(sizeof(ll_decoder_t));
    if ( decoder == NULL ) {
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }

    // prepare the decoder
    char buffer[UFR_ARGS_TOKEN];
    const char* sep = ufr_args_gets(args, buffer, "@sep", " ");
    decoder->sep = sep[0];
    link->dcr_obj = (void*) decoder;
    return UFR_OK;
}

static
void ufr_dcr_text_close(link_t* link) {
    if ( link->dcr_obj != NULL ) {
        free(link->dcr_obj);
        link->dcr_obj = NULL;
    }
}

static
int ufr_dcr_text_next(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    char token[TOKEN_SIZE];
    return lex_next_token(decoder, token);
}

static
char ufr_dcr_text_get_type(link_t* link) {
    // ll_decoder_t* decoder = link->dcr_obj;
    return 's';
}

static
int ufr_dcr_text_get_nbytes(link_t* link) {
    ll_decoder_t* decoder = link->dcr_obj;
    return decoder->msg_size;
}

static
int ufr_dcr_text_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
	ll_decoder_t* decoder = link->dcr_obj;

    // initialize the decoder object with the new line
    decoder->msg_ptr = msg_data;
    decoder->msg_size = msg_size;
    decoder->msg_index = 0;

    // ufr_dbg("%s", msg_data);
    return UFR_OK;
}

static
int ufr_dcr_text_get_u32(link_t* link, uint32_t* val, int max_nitems) {
    char token[TOKEN_SIZE];
    ll_decoder_t* decoder = link->dcr_obj;
    int copied = 0;
    for (; copied<max_nitems; copied++) {
        if ( lex_next_token(decoder, token) != UFR_OK ) {
            break;
        }
        val[copied] = (uint32_t) atoi(token);
    }
    return copied;
}

static
int ufr_dcr_text_get_i32(link_t* link, int32_t* val, int max_nitems) {
    char token[TOKEN_SIZE];
    ll_decoder_t* decoder = link->dcr_obj;
    int copied = 0;
    for (; copied<max_nitems; copied++) {
        if ( lex_next_token(decoder, token) != UFR_OK ) {
            break;
        }
        val[copied] = (int32_t) atoi(token);
    }
    return copied;
}

static
int ufr_dcr_text_get_f32(link_t* link, float* val, int max_nitems) {
    char token[TOKEN_SIZE];
    ll_decoder_t* decoder = link->dcr_obj;
    int copied = 0;
    for (; copied<max_nitems; copied++) {
        if ( lex_next_token(decoder, token) != UFR_OK ) {
            break;
        }
        val[copied] = (float) atof(token);
    }
    return copied;
}

static
int ufr_dcr_text_get_u64(link_t* link, uint64_t* val, int max_nitems) {
    char token[TOKEN_SIZE];
    ll_decoder_t* decoder = link->dcr_obj;
    int copied = 0;
    for (; copied<max_nitems; copied++) {
        if ( lex_next_token(decoder, token) != UFR_OK ) {
            break;
        }
        val[copied] = (uint64_t) atoi(token);
    }
    return copied;
}

static
int ufr_dcr_text_get_i64(link_t* link, int64_t* val, int max_nitems) {
    char token[TOKEN_SIZE];
    ll_decoder_t* decoder = link->dcr_obj;
    int copied = 0;
    for (; copied<max_nitems; copied++) {
        if ( lex_next_token(decoder, token) != UFR_OK ) {
            break;
        }
        val[copied] = atoi(token);
    }
    return copied;
}

static
int ufr_dcr_text_get_f64(link_t* link, double* val, int max_nitems) {
    char token[TOKEN_SIZE];
    ll_decoder_t* decoder = link->dcr_obj;
    int copied = 0;
    for (; copied<max_nitems; copied++) {
        if ( lex_next_token(decoder, token) != UFR_OK ) {
            break;
        }
        val[copied] = atof(token);
    }
    return copied;
}


static
int ufr_dcr_text_get_raw(link_t* link, uint8_t* out_val, int maxlen) {
	// ll_decoder_t* decoder = link->dcr_obj;
	return 0;
}

static
int ufr_dcr_text_get_str(link_t* link, char* out_val, int maxlen) {
	ll_decoder_t* decoder = link->dcr_obj;
    return lex_next_token(decoder, out_val);
}

static
ufr_dcr_api_t ufr_dcr_std_text_api = {
    .boot = ufr_dcr_text_boot,
    .close = ufr_dcr_text_close,

    // Receive
    .recv_cb = ufr_dcr_text_recv_cb,
    .recv_async_cb = ufr_dcr_text_recv_cb,

    // ignore
    .next = ufr_dcr_text_next,

    // metadata
    .get_type = NULL,
    .get_nbytes = ufr_dcr_text_get_nbytes,
    .get_nitems = NULL,
    .get_rawptr = NULL,

    // 32 bits
    .get_u32 = ufr_dcr_text_get_u32,
    .get_i32 = ufr_dcr_text_get_i32,
    .get_f32 = ufr_dcr_text_get_f32,

    // 64 bits
    .get_u64 = ufr_dcr_text_get_u64,
    .get_i64 = ufr_dcr_text_get_i64,
    .get_f64 = ufr_dcr_text_get_f64,

    // Binary and String
    .get_raw = ufr_dcr_text_get_raw,
    .get_str = ufr_dcr_text_get_str,

    // Enter and Leave
    .enter = NULL,
    .leave = NULL,
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_dcr_text_new(link_t* link) {
    link->dcr_api = &ufr_dcr_std_text_api;
	return UFR_OK;
}
