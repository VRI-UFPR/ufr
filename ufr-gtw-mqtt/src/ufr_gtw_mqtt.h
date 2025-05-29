/* BSD 2-Clause License
 * 
 * Copyright (c) 2025, Visao Robotica Imagem (VRI)
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

#pragma once

#define MQTT_QOS_0 0

typedef struct {
    uint16_t broker_port;
    char broker_hostname[128];
    char topic_name[128];
} ll_shr_t;

typedef struct {
    uint16_t broker_port;
    char broker_hostname[128];
    char topic1_name[128];
    char topic2_name[128];
} ll_shr_socket_t;

typedef struct {
    uint8_t start_type;
    volatile bool is_received;
    struct mosquitto* mosq;
    size_t msg_size;
    size_t msg_size_max;
    size_t msg_read_idx;
    char* msg_data;
    int32_t socket_msg_id;
} ll_obj_t;

extern size_t g_mosq_count;


int ufr_gtw_mqtt_new_client(link_t* link, int type);
int ufr_gtw_mqtt_new_server(link_t* link, int type);