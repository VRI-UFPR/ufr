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
//  Documentation
// ============================================================================

/*
fazer:
 - realocar o buffer da mensage quando receber mensagem maior que o buffer alocado

erros:
 - iniciar com publisher mas tentar receber dados
 - ufr_new cria um link temporario e nao pode ser guardado na estrutura
  - solucao foi colocar a inicializacao do cliente na funcao start
*/

// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <mosquitto.h>
#include <ufr.h>

#include "ufr_gtw_mqtt.h"

#define MQTT_MESSAGE_STATE_OK   1
#define MQTT_MESSAGE_STATE_END  0

extern volatile bool g_is_ok;

// ============================================================================
//  Subscriber Receive Callback
// ============================================================================

static
void ufr_gtw_mqtt_socket_recv_cb(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
    ll_obj_t* obj = (ll_obj_t*) userdata;

    // expand the buffer case payload is bigger
    if ( message->payloadlen > obj->msg_size_max ) {
        free(obj->msg_data);
        obj->msg_data = malloc(message->payloadlen);
        obj->msg_size_max = message->payloadlen;
    }

    // copy the message to the buffer
    obj->msg_size = message->payloadlen;
    obj->msg_read_idx = 0;
    memcpy(obj->msg_data, message->payload, obj->msg_size);
    obj->is_received = true;
}

// ============================================================================
//  Socket
// ============================================================================

static
int ufr_gtw_mqtt_socket_type(const link_t* link) {
    return 0;
}

static
int ufr_gtw_mqtt_socket_state(const link_t* link) {
    return 0;
}

static
size_t ufr_gtw_mqtt_socket_size(const link_t* link, int type) {
    return 0;
}

static
int ufr_gtw_mqtt_socket_boot (link_t* link, const ufr_args_t* args) {
    // initialize the mosquitto library on first time
    if ( g_mosq_count == 0 ) {
        mosquitto_lib_init();
    
        //Get libmosquitto version info
        int major, minor, revision;
        mosquitto_lib_version(&major, &minor, &revision);
        ufr_info(link, "Libmosquitto version: %d.%d.%d", major, minor, revision);
    }

    // prepare the shared object
    ll_shr_socket_t* shr = malloc(sizeof(ll_shr_socket_t));
    if ( shr == NULL ) {
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }

    // get the arguments
    ufr_args_gets(args, shr->broker_hostname, "@host", "127.0.0.1");
    shr->broker_port = ufr_args_geti(args, "@port", 1883);
    ufr_args_gets(args, shr->topic1_name, "@topic1", "topic1");
    ufr_args_gets(args, shr->topic2_name, "@topic2", "topic2");

    // prepare the private object
    ll_obj_t* obj = malloc(sizeof(ll_obj_t));
    if ( obj == NULL ) {
        free(shr);
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }

    obj->start_type = 0;
    obj->msg_size_max = 4096;
    obj->msg_data = malloc(obj->msg_size_max);
    if ( obj->msg_data == NULL ) {
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }

    obj->is_received = false;
    obj->mosq = NULL;

    // success
    link->gtw_shr = shr;
    link->gtw_obj = obj;
    g_mosq_count += 1;
    return UFR_OK;
}

static
int ufr_gtw_mqtt_socket_start (link_t* link, int type, const ufr_args_t* args) {
    ll_shr_socket_t* shr = link->gtw_shr;
    ll_obj_t* obj = link->gtw_obj;

    if ( type == UFR_START_SERVER ) {
        ufr_info(link, "starting server %s", shr->broker_hostname);

        // initialize the mosquitto client
        // *** iniatilize the client in the start function instead of boot
        obj->mosq = mosquitto_new(NULL, true, obj);
        if ( obj->mosq == NULL ) {
            return ufr_error(link, 1, "failed to create mosquitto client");
        }

        // connect the mosquitto client
        ufr_info(link, "connecting on %s:%d", shr->broker_hostname, shr->broker_port);
        if ( mosquitto_connect(obj->mosq, shr->broker_hostname, shr->broker_port, 60) != MOSQ_ERR_SUCCESS) {
            return ufr_error(link, 1, "connecting to MQTT broker failed");
        }
        obj->start_type = UFR_START_SERVER;

        ufr_info(link, "Subscribing on topic %s", shr->topic1_name);
        mosquitto_subscribe(obj->mosq, NULL, shr->topic1_name, 0);
        mosquitto_message_callback_set(obj->mosq, ufr_gtw_mqtt_socket_recv_cb);
        ufr_info(link, "connected");


    } else if ( type == UFR_START_CLIENT ) {
        ufr_info(link, "starting subscriber");

        // initialize the mosquitto client
        obj->mosq = mosquitto_new(NULL, true, obj);
        if ( obj->mosq == NULL ) {
            return ufr_error(link, 1, "failed to create mosquitto client");
        }

        // connect the mosquitto client
        ufr_info(link, "connecting on %s:%d", shr->broker_hostname, shr->broker_port);
        if ( mosquitto_connect(obj->mosq, shr->broker_hostname, shr->broker_port, 60) != MOSQ_ERR_SUCCESS ) {
            return ufr_error(link, 1, "connecting to MQTT broker failed");
        }

        // configure the subscriber
        ufr_info(link, "Subscribing on topic %s", shr->topic2_name);
        mosquitto_subscribe(obj->mosq, NULL, shr->topic2_name, 0);
        mosquitto_message_callback_set(obj->mosq, ufr_gtw_mqtt_socket_recv_cb);
        ufr_info(link, "connected");
        obj->start_type = UFR_START_CLIENT;
        obj->socket_msg_id = rand();
    }

    // success
    return UFR_OK;
}

static
void ufr_gtw_mqtt_socket_stop(link_t* link, int type) {
    if ( type == UFR_STOP_CLOSE ) {
        // free the private object
        ufr_info(link, "close link");
        ll_obj_t* obj = link->gtw_obj;
        mosquitto_destroy(obj->mosq);
        free(obj);
        link->gtw_obj = NULL;

        // free library
        if ( g_mosq_count == 1 ) {
            ufr_info(link, "closing the libmosquitto library");
            mosquitto_lib_cleanup();
            g_mosq_count = 0;
        } else {
            g_mosq_count -= 1;
        }
    }
}

static
int ufr_gtw_mqtt_server_recv(link_t* link) {
    ll_obj_t* obj = link->gtw_obj;

    // wait for the message
    const int timeout_ms = 250;
    while( obj->is_received == false ) {
        if ( g_is_ok == false ) {return -1;}
        mosquitto_loop(obj->mosq, timeout_ms, 1);
    }
    obj->is_received = false;

    // decoder the message
    // printf("recv %d bytes\n", obj->msg_size);
    if ( link->dcr_api != NULL ) {
        link->dcr_api->recv_cb(link, obj->msg_data, obj->msg_size);
    }

    int msg_id, msg_state;
    ufr_get(link, "dd", &msg_id, &msg_state);
    // printf("recv %d %d\n", msg_id, msg_state);
    obj->socket_msg_id = msg_id;

    return (msg_state == MQTT_MESSAGE_STATE_OK) ? UFR_OK : -1;
}

static
int ufr_gtw_mqtt_client_recv(link_t* link) {
    ll_obj_t* obj = link->gtw_obj;

    // wait for the message
    const int timeout_ms = 250;
    while( obj->is_received == false ) {
        if ( g_is_ok == false ) {return -1;}
        mosquitto_loop(obj->mosq, timeout_ms, 1);
    }
    obj->is_received = false;

    // decoder the message
    if ( link->dcr_api != NULL ) {
        link->dcr_api->recv_cb(link, obj->msg_data, obj->msg_size);
    }

    int msg_id, msg_state;
    ufr_get(link, "dd", &msg_id, &msg_state);
    // printf("cli recv %d %d\n", msg_id, msg_state);

    return (msg_state == MQTT_MESSAGE_STATE_OK) ? UFR_OK : -1;
}

static
int ufr_gtw_mqtt_socket_recv_async(link_t* link) {
    ll_obj_t* obj = link->gtw_obj;

    // wait for the message
    const int timeout_ms = 50;
    mosquitto_loop(obj->mosq, timeout_ms, 1);

    // Case received a message
    if ( obj->is_received == true ) {
        obj->is_received = false;

        // decoder the message
        if ( link->dcr_api != NULL ) {
            link->dcr_api->recv_cb(link, obj->msg_data, obj->msg_size);
        }

        // there is a message
        return UFR_OK;
    }

    // No message
    return -1;
}

static
size_t ufr_gtw_mqtt_socket_read(link_t* link, char* buffer, size_t max_size) {
    ll_obj_t* obj = link->gtw_obj;

    if ( obj == NULL || obj->msg_data == NULL ) {
        return 0;
    }
    if ( obj->msg_read_idx >= obj->msg_size ) {
        return 0;
    }

    const size_t rest = obj->msg_size - obj->msg_read_idx;
    if ( max_size > rest ) {
        memcpy(buffer, &obj->msg_data[obj->msg_read_idx], rest);
        obj->msg_read_idx += rest;
        return rest;
    }

    memcpy(buffer, &obj->msg_data[obj->msg_read_idx], max_size);
    obj->msg_read_idx += max_size;
    return max_size;
}

static
size_t ufr_gtw_mqtt_server_write(link_t* link, const char* buffer, size_t size) {
    ll_shr_socket_t* shr = link->gtw_shr;
    ll_obj_t* obj = link->gtw_obj;

    if ( obj->mosq == NULL ) {
        return ufr_error(link, 1, "Mosquisto pointer is null");
    }
    
    // printf("server_mqtt %d\n", link->state);
    if ( link->state == UFR_STATE_SEND_LAST ) {
        ufr_info(link, "last writing %ld bytes on %s", size, shr->topic2_name);
        const int error = mosquitto_publish(obj->mosq, NULL, shr->topic2_name, size, buffer, MQTT_QOS_0, false);
        if ( error != MOSQ_ERR_SUCCESS ) {
            return ufr_error(link, 0, "error");
        }

        link->state = UFR_STATE_SEND;
        const int val[2] = {obj->socket_msg_id, MQTT_MESSAGE_STATE_END};
        link->enc_api->clear(link);
        link->enc_api->put_i32(link, val, 2);
        link->enc_api->put_cmd(link, '\n');
        link->state = UFR_STATE_SEND_LAST;
        
    } else {
        ufr_info(link, "writing %ld bytes on %s", size, shr->topic2_name);
        const int error = mosquitto_publish(obj->mosq, NULL, shr->topic2_name, size, buffer, MQTT_QOS_0, false);
        if ( error != MOSQ_ERR_SUCCESS ) {
            return ufr_error(link, 0, "error");
        }
    }
    return size;
}

static
size_t ufr_gtw_mqtt_client_write(link_t* link, const char* buffer, size_t size) {
    ll_shr_socket_t* shr = link->gtw_shr;
    ll_obj_t* obj = link->gtw_obj;

    if ( obj->mosq == NULL ) {
        return ufr_error(link, 1, "aaa");
    }

    if ( link->state == UFR_STATE_SEND_LAST ) {
        ufr_info(link, "last writing %ld bytes on %s", size, shr->topic1_name);
        const int error = mosquitto_publish(obj->mosq, NULL, shr->topic1_name, size, buffer, MQTT_QOS_0, false);
        if ( error != MOSQ_ERR_SUCCESS ) {
            return ufr_error(link, 0, "error");
        }

        link->state = UFR_STATE_SEND;
        const int val[2] = {obj->socket_msg_id, MQTT_MESSAGE_STATE_END};
        link->enc_api->clear(link);
        link->enc_api->put_i32(link, val, 2);
        link->enc_api->put_cmd(link, '\n');
        link->state = UFR_STATE_SEND_LAST;

    } else {
        ufr_info(link, "writing %ld bytes on %s", size, shr->topic1_name);
        const int error = mosquitto_publish(obj->mosq, NULL, shr->topic1_name, size, buffer, MQTT_QOS_0, false);
        if ( error != MOSQ_ERR_SUCCESS ) {
            return ufr_error(link, 0, "error");
        }
    }
    return size;
}

static
int ufr_gtw_mqtt_server_ready(link_t* link) {
    // ll_shr_socket_t* shr = link->gtw_shr;
    // printf("opa\n");

    ll_obj_t* obj = link->gtw_obj;
    ufr_put(link, "dd", obj->socket_msg_id, 1);

    return UFR_OK;
}

static
int ufr_gtw_mqtt_client_ready(link_t* link) {
    // ll_shr_socket_t* shr = link->gtw_shr;
    // printf("opa\n");

    // put in the message the field id and state OK
    ll_obj_t* obj = link->gtw_obj;
    ufr_put(link, "dd", obj->socket_msg_id, 1);

    return UFR_OK;
}

static
ufr_gtw_api_t ufr_gtw_mqtt_client_api = {
    .name = "mqtt/client",
    .type = ufr_gtw_mqtt_socket_type,
    .state = ufr_gtw_mqtt_socket_state,
    .size = ufr_gtw_mqtt_socket_size,
    .boot = ufr_gtw_mqtt_socket_boot,
    .start = ufr_gtw_mqtt_socket_start,
    .stop = ufr_gtw_mqtt_socket_stop,
    .copy = NULL,
    .recv = ufr_gtw_mqtt_client_recv,
    .recv_async = ufr_gtw_mqtt_socket_recv_async,
    .read = ufr_gtw_mqtt_socket_read,
    .write = ufr_gtw_mqtt_client_write,
    .ready = ufr_gtw_mqtt_client_ready,
};

static
ufr_gtw_api_t ufr_gtw_mqtt_server_api = {
    .name = "mqtt/server",
    .type = ufr_gtw_mqtt_socket_type,
    .state = ufr_gtw_mqtt_socket_state,
    .size = ufr_gtw_mqtt_socket_size,
    .boot = ufr_gtw_mqtt_socket_boot,
    .start = ufr_gtw_mqtt_socket_start,
    .stop = ufr_gtw_mqtt_socket_stop,
    .copy = NULL,
    .recv = ufr_gtw_mqtt_server_recv,
    .recv_async = ufr_gtw_mqtt_socket_recv_async,
    .read = ufr_gtw_mqtt_socket_read,
    .write = ufr_gtw_mqtt_server_write,
    .ready = ufr_gtw_mqtt_server_ready
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_mqtt_new_client(link_t* link, int type) {
    ufr_link_init(link, &ufr_gtw_mqtt_client_api);
    return UFR_OK;
}

int ufr_gtw_mqtt_new_server(link_t* link, int type) {
    ufr_link_init(link, &ufr_gtw_mqtt_server_api);
    return UFR_OK;
}
