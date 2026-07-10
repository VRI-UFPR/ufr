/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Visao Robotica Imagem (VRI)
 *   Felipe Bombardelli
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
 * */

// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <telebot.h>
#include <ufr.h>

typedef struct {
    FILE* gnuplot_pipe;

} gtw_obj_t;

typedef struct {
    int a;
} ll_decoder_t;


// ============================================================================
//  Encoder
// ============================================================================

int ufr_enc_gnuplot_boot(link_t* link, const ufr_args_t* args) {
    link->enc_obj = ufr_buffer_new();
    return (link->enc_obj!=NULL) ? UFR_OK : -1;
}

void ufr_enc_gnuplot_close(link_t* link) {
    if ( link->enc_obj ) {
        free(link->enc_obj);
    }
}

int ufr_enc_gnuplot_clear(link_t* link) {
    ufr_buffer_t* buffer = link->enc_obj;
    ufr_buffer_clear(buffer);
    return UFR_OK;
}

int ufr_enc_gnuplot_put_u32(link_t* link, const uint32_t* val, int nitems) {
    int wrote = 0;
    ufr_buffer_t* buffer = link->enc_obj;
    for (;wrote<nitems; wrote++) {
        ufr_buffer_put_i32_as_str(buffer, val[wrote]);
    }
    return wrote;
}

int ufr_enc_gnuplot_put_i32(link_t* link, const int32_t* val, int nitems) {
    int wrote = 0;
    ufr_buffer_t* buffer = link->enc_obj;
    for (;wrote<nitems; wrote++) {
        ufr_buffer_put_i32_as_str(buffer, val[wrote]);
    }
    return wrote;
}

int ufr_enc_gnuplot_put_f32(link_t* link, const float* val, int nitems) {
    int wrote = 0;
    ufr_buffer_t* buffer = link->enc_obj;
    for (;wrote<nitems; wrote++) {
        ufr_buffer_put_f32_as_str(buffer, val[wrote]);
    }
    return wrote;
}

int ufr_enc_gnuplot_put_u64(link_t* link, const uint64_t* val, int nitems) {
    int wrote = 0;
    ufr_buffer_t* buffer = link->enc_obj;
    for (;wrote<nitems; wrote++) {
        ufr_buffer_put_i32_as_str(buffer, val[wrote]);
    }
    return wrote;
}

int ufr_enc_gnuplot_put_i64(link_t* link, const int64_t* val, int nitems) {
    int wrote = 0;
    ufr_buffer_t* buffer = link->enc_obj;
    for (;wrote<nitems; wrote++) {
        ufr_buffer_put_i32_as_str(buffer, val[wrote]);
    }
    return wrote;
}

int ufr_enc_gnuplot_put_f64(link_t* link, const double* val, int nitems) {
    int wrote = 0;
    ufr_buffer_t* buffer = link->enc_obj;
    for (;wrote<nitems; wrote++) {
        ufr_buffer_put_i32_as_str(buffer, val[wrote]);
    }
    return wrote;
}

int ufr_enc_gnuplot_put_str(link_t* link, const char* val) {
    ufr_buffer_t* buffer = link->enc_obj;
    // ufr_buffer_put_str(buffer, val);
    return UFR_OK;
}

int ufr_enc_gnuplot_cmd_send(link_t* link) {
    gtw_obj_t* obj = link->gtw_obj;
    ufr_buffer_t* buffer = link->enc_obj;
    // const telebot_error_e ret = telebot_send_message(obj->handle, obj->message.chat->id, buffer->ptr, "HTML", false, false, 0, "");
    ufr_buffer_clear(buffer);
    ufr_info(link, "Sent message");
    return 0;
}

int ufr_enc_gnuplot_cmd_eof(link_t* link) {
printf("aab\n");
return UFR_OK;
    gtw_obj_t* obj = link->gtw_obj;
    obj->req_offset = obj->updates[obj->req_index].update_id + 1;
    obj->req_index += 1;
    const telebot_error_e ret = telebot_put_updates(obj->updates, obj->req_index);
    ufr_info(link, "End of answer");
    return ret;
}



int ufr_enc_gnuplot_enter(link_t* link, size_t maxsize) {
    return UFR_OK;
}

int ufr_enc_gnuplot_leave(link_t* link) {
    return UFR_OK;
}

ufr_enc_api_t ufr_enc_gnuplot_api = {
    .init = ufr_enc_gnuplot_boot,
    .free = ufr_enc_gnuplot_close,

    .put_u32 = ufr_enc_gnuplot_put_u32,
    .put_i32 = ufr_enc_gnuplot_put_i32,
    .put_f32 = ufr_enc_gnuplot_put_f32,

    .put_u64 = ufr_enc_gnuplot_put_u64,
    .put_i64 = ufr_enc_gnuplot_put_i64,
    .put_f64 = ufr_enc_gnuplot_put_f64,

    .put_str = ufr_enc_gnuplot_put_str,
    .put_raw = NULL,
    .put_bin = NULL,

    // Commands
    .cmd_enter = ufr_enc_gnuplot_enter,
    .cmd_leave = ufr_enc_gnuplot_leave,
    .cmd_next = NULL,
    .cmd_clear = ufr_enc_gnuplot_clear,
    .cmd_send = ufr_enc_gnuplot_cmd_send,
    .cmd_eof = ufr_enc_gnuplot_cmd_eof
};

// ============================================================================
//  Gateway Socket
// ============================================================================

static
int ufr_gtw_gnuplot_type(const link_t* link) {
    return 0;
}

static
int ufr_gtw_gnuplot_state(const link_t* link) {
    return 0;
}

static
size_t ufr_gtw_gnuplot_size(const link_t* link, int type) {
    return 0;
}

static
int ufr_gtw_gnuplot_boot (link_t* link, const ufr_args_t* args) {
    FILE *gnuplot_pipe = popen("gnuplot", "w");

    if (gnuplot_pipe == NULL) {
        fprintf(stderr, "Error: Could not open pipe to gnuplot.\n");
        return 1;
    }

    // Configure gnuplot settings via the pipe
    fprintf(gnuplot_pipe, "set title 'Real-Time Data Stream'\n");
    fprintf(gnuplot_pipe, "set xlabel 'X Axis'\n");
    fprintf(gnuplot_pipe, "set ylabel 'Y Axis'\n");
    fprintf(gnuplot_pipe, "set yrange [-1.5:1.5]\n");

    // Success
    gtw_obj_t* gtw_obj = malloc(sizeof(obj_t));
    gtw_obj->gnuplot_pipe = gnuplot_pipe;
    link->gtw_obj = gtw_obj;
    return UFR_OK;
}

static
int ufr_gtw_gnuplot_start (link_t* link, int type, const ufr_args_t* args) {
    gtw_obj_t* obj = link->gtw_obj;

    if ( type == UFR_START_PUBLISHER ) {

    } else {
        return -1;
    }

    // success
    return UFR_OK;
}

static
void ufr_gtw_gnuplot_stop(link_t* link, int type) {
    gtw_obj_t* obj = link->gtw_obj;

}

static
int ufr_gtw_gnuplot_recv(link_t* link) {
    return -1;
}

static
int ufr_gtw_gnuplot_recv_async(link_t* link) {
    return -1;
}

static
size_t ufr_gtw_gnuplot_read(link_t* link, char* buffer, size_t max_size) {
    return -1;
}

static
size_t ufr_gtw_gnuplot_write(link_t* link, const char* buffer, size_t size) {
    gtw_obj_t* obj = link->gtw_obj;
     return fwrite(buffer, size, 1, obj->gnuplot_pipe);
}

static
int ufr_gtw_gnuplot_ready(link_t* link) {
    return UFR_OK;
}

static
ufr_gtw_api_t ufr_gtw_gnuplot_api = {
    .name = "gnuplot",
    .type = ufr_gtw_gnuplot_type,
    .state = ufr_gtw_gnuplot_state,
    .size = ufr_gtw_gnuplot_size,
    .boot = ufr_gtw_gnuplot_boot,
    .start = ufr_gtw_gnuplot_start,
    .stop = ufr_gtw_gnuplot_stop,
    .copy = NULL,
    .recv = ufr_gtw_gnuplot_recv,
    .recv_async = ufr_gtw_gnuplot_recv_async,
    .read = ufr_gtw_gnuplot_read,
    .write = ufr_gtw_gnuplot_write,
    .ready = ufr_gtw_gnuplot_ready,
};

// ============================================================================
//  Publico
// ============================================================================

int ufr_gtw_gnuplot_new(link_t* link, int type) {
    link->gtw_api = &ufr_gtw_gnuplot_api;
    return UFR_OK;
}

