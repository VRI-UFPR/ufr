/* BSD 2-Clause License
 * 
 * Copyright (c) 2024, Visao Robotica e Imagem (VRI)
 *  - Felipe Bombardelli <felipebombardelli@gmail.com>
 *  - Dayane Oliveira Carvalho
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ufr.h"

int g_contador = 0;
uint8_t g_default_log_level = 5;
volatile bool g_is_ok = true;

typedef int (*loop_callback)(void);
uint8_t g_callback_array_count = 0;
loop_callback g_callback_array[5] = {NULL, NULL, NULL, NULL, NULL};

// ============================================================================
//  Outros
// ============================================================================

const char* ufr_api_name(const link_t* link) {
    if ( link == NULL || link->gtw_api == NULL ) {
        return "None";
    }
    return link->gtw_api->name;
}

// ============================================================================
//  UFR LOOP
// ============================================================================

int ufr_loop_put_callback( int (*loop_callback)(void)  ) {
    if ( g_callback_array_count < 5 ) {
        g_callback_array[ g_callback_array_count ] = loop_callback;
        g_callback_array_count += 1;
        return UFR_OK;
    }
    return -1;
}

bool ufr_loop_ok() {
    for (uint8_t i=0; i<g_callback_array_count; i++) {
        if ( g_callback_array[i]() != UFR_OK ) {
            return false;
        }
    }
    return g_is_ok;
}

void ufr_loop_set_end() {
    g_is_ok = false;
}

// ============================================================================
//  UFR LINK
// ============================================================================

int ufr_link_state(const link_t* link) {
    return link->type_started;
}

bool ufr_link_is_publisher(const link_t* link) {
    return link->type_started == UFR_START_PUBLISHER;
}

bool ufr_link_is_subscriber(const link_t* link) {
    return link->type_started == UFR_START_SUBSCRIBER;
}

bool ufr_link_is_server(const link_t* link) {
    return link->type_started == UFR_START_SERVER;
}

bool ufr_link_is_client(const link_t* link) {
    return link->type_started == UFR_START_CLIENT;
}

bool ufr_link_is_valid(const link_t* link) {
    return link->gtw_api != NULL;
}

bool ufr_link_is_blank(const link_t* link) {
    return link->gtw_api == NULL;
}

void ufr_link_init(link_t* link, ufr_gtw_api_t* gtw_api) {
    link->gtw_api = gtw_api;
    link->gtw_obj = NULL;
    link->gtw_shr = NULL;
    link->dcr_api = NULL;
    link->dcr_obj = NULL;
    link->enc_api = NULL;
    link->enc_obj = NULL;
    link->type_started = UFR_START_BLANK;
    link->status = UFR_STATUS_RESET;
    link->put_count = 0;
    link->log_level = g_default_log_level;
}

// ============================================================================
//  UFR RECV
// ============================================================================

int ufr_recv(link_t* link) {
    ufr_log_ini(link, "receiving data from link");
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }
    if ( link->log_level > 0 ) {
        if ( link->gtw_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->gtw_api->recv == NULL ) {
            ufr_fatal(link, 1, "gtw_api->recv is null");
        }
    }
    
    link->state = 1;
    const int retval = link->gtw_api->recv(link);
    ufr_log_end(link, "received data from link %d", retval);
    return retval;
}

int ufr_recv_async(link_t* link) {
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }
    if ( link->log_level > 0 ) {
        if ( link->gtw_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->gtw_api->recv_async == NULL ) {
            ufr_fatal(link, 1, "gtw_api->recv_async is null");
        }
    }

    return link->gtw_api->recv_async(link);
}

bool ufr_send(link_t* link) {
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }
    if ( link->log_level > 0 ) {
        if ( link->enc_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->enc_api->put_cmd == NULL ) {
            ufr_fatal(link, 1, "enc_api->put_cmd is null");
        }
    }

    int error = link->enc_api->put_cmd(link, '\n');
    link->put_count = 0;
    return error == UFR_OK;
}

void ufr_close(link_t* link) {
    if ( link == NULL ) {
        ufr_warn(link, 1, "link is null");
        return;
    }

    // free Encoder
    if ( link->enc_api != NULL && link->enc_api->close != NULL ) {
        ufr_info(link, "Free Encoder");
        link->enc_api->close(link);
        link->enc_api = NULL;
        link->enc_obj = NULL;
    }

    // free Decoder
    if ( link->dcr_api != NULL && link->dcr_api->close != NULL ) {
        ufr_info(link, "Free Decoder");
        link->dcr_api->close(link);
        link->dcr_api = NULL;
        link->dcr_obj = NULL;
    }

    // free Gateway
    if ( link->gtw_api != NULL && link->gtw_api->stop != NULL ) {
        ufr_info(link, "Free Gateway");
        link->gtw_api->stop(link, UFR_STOP_CLOSE);
        link->gtw_api = NULL;
        link->gtw_obj = NULL;
    }

    link->is_started = 0;
    link->is_booted = 0;
}

int ufr_boot_enc(link_t* link, const ufr_args_t* args) {
    return link->enc_api->boot(link, args);
}

int ufr_boot_dcr(link_t* link, const ufr_args_t* args) {
    return link->dcr_api->boot(link, args);
}

// ============================================================================
//  Link - Character Stream
// ============================================================================

size_t ufr_read(link_t* link, char* buffer, size_t maxsize) {
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }
    if ( link->log_level > 0 ) {
        if ( link->gtw_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->gtw_api->read == NULL ) {
            ufr_fatal(link, 1, "gtw_api->read is null");
        }
    }

    return link->gtw_api->read(link, buffer, maxsize);
}

size_t ufr_write(link_t* link, const char* buffer, size_t size) {
    if ( link == NULL ) {
        ufr_fatal(link, 1, "link is null");
    }

    // With Debug
    if ( link->log_level > 0 ) {
        if ( link->gtw_api == NULL ) {
            ufr_fatal(link, 1, "gtw_api is null");
        } else if ( link->gtw_api->read == NULL ) {
            ufr_fatal(link, 1, "gtw_api->write is null");
        }

        const size_t wrote = link->gtw_api->write(link, buffer, size);
        ufr_log(link, "sent %ld bytes", wrote);
        return wrote;

    // No Debug
    } else {
    	return link->gtw_api->write(link, buffer, size);
    }
}

// ============================================================================
//  UFR DUMMY
// ============================================================================

size_t  ufr_dummy_read(link_t* link, char* buffer, size_t size) {
    return 0;
}

size_t ufr_dummy_write(link_t* link, const char* buffer, size_t size) {
    return 0;
}

bool ufr_dummy_recv(link_t* link) {
    return false;
}

int ufr_dummy_send(link_t* link) {
    return 0;
}

link_t ufr_accept(link_t* link) {
    link_t client;
    link->gtw_api->accept(link, &client);
    return client;
}

int ufr_recv_peer_name(link_t* link, char* buffer, size_t maxbuffer) {
    return link->gtw_api->recv_peer_name(link, buffer, maxbuffer);
}

// ============================================================================
//  Use for tests
// ============================================================================

uint32_t g_ufr_test_count = 0;

void ufr_test_inc_count() {
    g_ufr_test_count += 1;
}

void ufr_test_print_result() {
    printf("OK - %d tests passed\n", g_ufr_test_count);
} 

// ============================================================================
//  UFR APP
// ============================================================================

ufr_node_t* g_app_root = NULL;

int ufr_app_init(ufr_node_t* root) {
    if ( root == NULL ) {
        return -1;
    }

    if ( g_app_root != NULL ) {
        return -2;
    }

    g_app_root = root;
    return UFR_OK;
}


int ufr_app_open(link_t* link, const char* name, int type) {
    if ( g_app_root == NULL ) {
        return -1;
    }
    ufr_link_init(link, NULL);

    for (int i=0; i<32; i++) {
        if ( g_app_root[i].name == NULL ) {
            break;
        }

        if ( strcmp(name, g_app_root[i].name) == 0 ) {
            if ( type == UFR_START_SUBSCRIBER ) {
                ufr_subscriber_args(link, &g_app_root[i].args);
            } else if ( type == UFR_START_PUBLISHER ) {
                ufr_publisher_args(link, &g_app_root[i].args);
            }
            // return sys_ufr_new_link(link, type, &g_app_root[i].args);
            return -1;
        }
    }
    // Error: not found
    return -2;
}

link_t ufr_app_publisher(const char* name) {
    link_t link;
    ufr_app_open(&link, name, UFR_START_PUBLISHER);
    return link;
}

link_t ufr_app_subscriber(const char* name) {
    link_t link;
    ufr_app_open(&link, name, UFR_START_SUBSCRIBER);
    return link;
}

link_t ufr_app_client(const char* name) {
    link_t link;
    ufr_app_open(&link, name, UFR_START_CLIENT);
    return link;
}

link_t ufr_app_server(const char* name) {
    link_t link;
    ufr_app_open(&link, name, UFR_START_SERVER_ST);
    return link;
}

// ============================================================================
//  UFR LOG
// ============================================================================

void ufr_log_put(link_t* link, uint8_t level, const char* func_name, const char* format, ...) {
    if ( level > link->log_level ) {
        return;
    }
    va_list list;
    va_start(list, format);
    const char* name = ufr_api_name(link);
    fprintf(stderr, "# info: %s: ", func_name);
    vfprintf(stderr, format, list);
    fprintf(stderr, "\n");
    va_end(list);
}

int ufr_log_put_error(link_t* link, int error, const char* func_name, const char* format, ...) {
    // copy the error message in the link buffer
    va_list list;
    va_start(list, format);
    vsnprintf(&link->errstr[0], sizeof(link->errstr), format, list);
    va_end(list);

    // show the debug
    if ( link->log_level > 0 ) {
        fprintf(stderr, "\x1B[31m# erro: %s\033[0m: ", func_name);
        fprintf(stderr, "%s", &link->errstr[0]);
        fprintf(stderr, "\n");
    }

    // return error number
    return error;
}

void ufr_log_put_fatal(int error, const char* func_name, const char* format, ...) {
    // copy the error message in the link buffer
    va_list list;
    va_start(list, format);

    // show the debug
    fprintf(stderr, "\x1B[31m# erro: %s\033[0m: ", func_name);
    vfprintf(stderr, format, list);
    fprintf(stderr, "\n");

    va_end(list);
    exit(error);
}

// ============================================================================
//  UFR
// ============================================================================

link_t ufr_subscriber(const char* format, ...) {
    link_t link;
    ufr_link_init(&link, NULL);

    // load variable arguments to args
    ufr_args_t args;
    va_list list;
    va_start(list, format);
    ufr_args_load_from_va(&args, format, list);
    va_end(list);

    // Open link
    ufr_subscriber_args(&link, &args);

    // success
    return link;
}

int ufr_subscriber_args(link_t* link, const ufr_args_t* args) {
    // Prepare Gateway
    ufr_log(link, "preparing the Gateway");
    int(*func_gtw_new)(link_t*,int) = ufr_args_getfunc(args, "gtw", "@new", NULL);
    if ( func_gtw_new == NULL ) {
        ufr_fatal(link, 1, "erro1");
    }
    if ( func_gtw_new(link, UFR_START_SUBSCRIBER) != UFR_OK ) {
        ufr_fatal(link, 1, "erro2");
    }

    // Case no API was defined by driver function
    if ( link->gtw_api == NULL ) {
        ufr_fatal(link, 1, "erro3");
    }

    if ( link->gtw_api->boot == NULL ) {
        ufr_fatal(link, 1, "erro3");
    }

    //
    link->type_started = UFR_START_SUBSCRIBER;
    if ( link->gtw_api->boot(link, args) != UFR_OK ) {
        ufr_fatal(link, 1, "erro4");
    }

    // Prepare Decoder
    if ( link->dcr_api == NULL ) {
        ufr_log(link, "preparing the Decoder");
        int(*func_dcr_new)(link_t*,int) = ufr_args_getfunc(args, "dcr", "@coder", NULL);
        if ( func_dcr_new == NULL ) {
            ufr_dcr_sys_new_std(link, UFR_START_SUBSCRIBER);
        } else {
            if ( func_dcr_new(link, UFR_START_SUBSCRIBER) != UFR_OK ) {
                ufr_fatal(link, 1, "erro5");
            }
        }
        if ( link->dcr_api->boot != NULL ) {
            if ( link->dcr_api->boot(link, args) != UFR_OK ) {
                ufr_fatal(&link, 1, "erro6");
            }
        }
    }

    // start
    ufr_log(&link, "starting the link");
    if ( link->gtw_api->start != NULL ) {
        if ( link->gtw_api->start(link, UFR_START_SUBSCRIBER, args) != UFR_OK ) {
            ufr_fatal(link, 1, "erro7");
        }
    }

    // success
    return UFR_OK;
}

link_t ufr_publisher(const char* format, ...) {
    link_t link;

    // load variable arguments to args
    ufr_args_t args;
    va_list list;
    va_start(list, format);
    ufr_args_load_from_va(&args, format, list);
    va_end(list);

    // open the link
    ufr_publisher_args(&link, &args);

    // success
    return link;
}

int ufr_publisher_args(link_t* link, const ufr_args_t* args) {
    // Prepare Gateway
    int(*func_gtw_new)(link_t*,int) = ufr_args_getfunc(args, "gtw", "@new", NULL);
    if ( func_gtw_new == NULL ) {
        ufr_fatal(link, 1, "erro1");
    }
    if ( func_gtw_new(link, UFR_START_PUBLISHER) != UFR_OK ) {
        ufr_fatal(link, 1, "erro2");
    }

    link->type_started = UFR_START_PUBLISHER;
    if ( link->gtw_api->boot(link, args) != UFR_OK ) {
        ufr_fatal(link, 1, "erro3");
    }

    // Prepare Encoder
    if ( link->enc_api == NULL ) {
        int(*func_enc_new)(link_t*,int) = ufr_args_getfunc(args, "enc", "@coder", NULL);
        if ( func_enc_new == NULL ) {
            ufr_enc_sys_new_std(link, UFR_START_PUBLISHER);
        } else {
            if ( func_enc_new(link, UFR_START_PUBLISHER) != UFR_OK ) {
                ufr_fatal(link, 1, "erro4");
            }
        }

        if ( link->enc_api->boot(link, args) != UFR_OK ) {
            ufr_fatal(link, 1, "erro5");
        }
    }

    // start
    if ( link->gtw_api->start(link, UFR_START_PUBLISHER, args) != UFR_OK ) {
        ufr_fatal(link, 1, "erro6");
    }

    // success
    return UFR_OK;
}


link_t ufr_client(const char* format, ...) {
    link_t link;

    // load variable arguments to args
    ufr_args_t args;
    va_list list;
    va_start(list, format);
    ufr_args_load_from_va(&args, format, list);
    va_end(list);

    // Prepare Gateway
    int(*func_gtw_new)(link_t*,int) = ufr_args_getfunc(&args, "gtw", "@new", NULL);
    if ( func_gtw_new == NULL ) {
        ufr_fatal(&link, 1, "erro1");
    }
    if ( func_gtw_new(&link, UFR_START_CLIENT) != UFR_OK ) {
        ufr_fatal(&link, 1, "erro2");
    }

    link.type_started = UFR_START_CLIENT;
    if ( link.gtw_api->boot(&link, &args) != UFR_OK ) {
        ufr_fatal(&link, 1, "erro3");
    }

    // Prepare Decoder
    if ( link.dcr_api == NULL ) {
        int(*func_dcr_new)(link_t*,int) = ufr_args_getfunc(&args, "dcr", "@coder", NULL);
        if ( func_dcr_new == NULL ) {
            ufr_dcr_sys_new_std(&link, UFR_START_CLIENT);
        } else {
            if ( func_dcr_new(&link, UFR_START_CLIENT) != UFR_OK ) {
                ufr_fatal(&link, 1, "erro4");
            }
        }

        if ( link.dcr_api->boot(&link, &args) != UFR_OK ) {
            ufr_fatal(&link, 1, "erro5");
        }
    }

    // Prepare Encoder
    if ( link.enc_api == NULL ) {
        int(*func_enc_new)(link_t*,int) = ufr_args_getfunc(&args, "enc", "@coder", NULL);
        if ( func_enc_new == NULL ) {
            ufr_enc_sys_new_std(&link, UFR_START_CLIENT);
        } else {
            if ( func_enc_new(&link, UFR_START_CLIENT) != UFR_OK ) {
                ufr_fatal(&link, 1, "erro4");
            }
        }

        if ( link.enc_api->boot(&link, &args) != UFR_OK ) {
            ufr_fatal(&link, 1, "erro5");
        }
    }

    // start
    if ( link.gtw_api->start(&link, UFR_START_CLIENT, &args) != UFR_OK ) {
        ufr_fatal(&link, 1, "erro6");
    }

    // success
    return link;
}

link_t ufr_server_st(const char* format, ...) {
    link_t link;

    // load variable arguments to args
    ufr_args_t args;
    va_list list;
    va_start(list, format);
    ufr_args_load_from_va(&args, format, list);
    va_end(list);

    // Prepare Gateway
    int(*func_gtw_new)(link_t*,int) = ufr_args_getfunc(&args, "gtw", "@new", NULL);
    if ( func_gtw_new == NULL ) {
        ufr_fatal(&link, 1, "erro1");
    }
    if ( func_gtw_new(&link, UFR_START_SERVER) != UFR_OK ) {
        ufr_fatal(&link, 1, "erro2");
    }

    link.type_started = UFR_START_SERVER;
    if ( link.gtw_api->boot(&link, &args) != UFR_OK ) {
        ufr_fatal(&link, 1, "erro3");
    }

    // Prepare Decoder
    if ( link.dcr_api == NULL ) {
        int(*func_dcr_new)(link_t*,int) = ufr_args_getfunc(&args, "dcr", "@coder", NULL);
        if ( func_dcr_new == NULL ) {
            ufr_dcr_sys_new_std(&link, UFR_START_SERVER);
        } else {
            if ( func_dcr_new(&link, UFR_START_SERVER) != UFR_OK ) {
                ufr_fatal(&link, 1, "erro4");
            }
        }

        if ( link.dcr_api->boot(&link, &args) != UFR_OK ) {
            ufr_fatal(&link, 1, "erro5");
        }
    }

    // Prepare Encoder
    if ( link.enc_api == NULL ) {
        int(*func_enc_new)(link_t*,int) = ufr_args_getfunc(&args, "enc", "@coder", NULL);
        if ( func_enc_new == NULL ) {
            ufr_enc_sys_new_std(&link, UFR_START_SERVER);
        } else {
            if ( func_enc_new(&link, UFR_START_SERVER) != UFR_OK ) {
                ufr_fatal(&link, 1, "erro4");
            }
        }

        if ( link.enc_api->boot(&link, &args) != UFR_OK ) {
            ufr_fatal(&link, 1, "erro5");
        }
    }

    // start
    if ( link.gtw_api->start(&link, UFR_START_SERVER, &args) != UFR_OK ) {
        ufr_fatal(&link, 1, "erro6");
    }

    // success
    return link;
}


void ufr_exit_if_error(link_t* link) {
    if ( link == NULL ) {
        printf("Link com error\n");
        exit(1);
    }
    if ( link->type_started == 0 ) {
        printf("Link com error\n");
        exit(1);
    }
}