/* BSD 2-Clause License
 * 
 * Copyright (c) 2024, Visao Robotica e Imagem (VRI)
 *  - Felipe Bombardelli <felipebombardelli@gmail.com>
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
#include <ufr.h>

#include "rclcpp/rclcpp.hpp"
#include "ufr_gtw_ros2.hpp"


#include "{{type_name.lower()}}.hpp"

#define message_t               {{type_name_cpp}}

struct ll_encoder_t {
    rclcpp::Publisher<message_t>::SharedPtr publisher;
    message_t message;
    int index;
    int index1;
};

// ============================================================================
//  String Message Driver
// ============================================================================

static
int ufr_enc_ros2_init(link_t* link, const ufr_args_t* args) {
    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;
    ll_encoder_t* enc_obj = new ll_encoder_t();
    enc_obj->publisher = gtw_obj->m_node->create_publisher<message_t>("topic", 10);
    link->enc_obj = enc_obj;
    return UFR_OK;
}

static
void ufr_enc_ros2_free(link_t* link) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {
        free(enc_obj);
        link->enc_obj = NULL;
    }
}

static
int ufr_enc_ros2_put_u32(link_t* link, const uint32_t val[], int nitems) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
        {% for field in fields -%}

            {% if field.type.type_id in [10,11] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id in [3,5,7,9, 2,4,6,8] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id == 17 %}
            case {{field.index}} : enc_obj->message.{{field.name}} = "";
            {%- endif %}

        {%- endfor %}
        }
    }
    return 0;
}

static
int ufr_enc_ros2_put_i32(link_t* link, const int32_t val[], int nitems) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
        {% for field in fields -%}

            {% if field.type.type_id in [10,11] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id in [3,5,7,9, 2,4,6,8] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id == 17 %}
            case {{field.index}} : enc_obj->message.{{field.name}} = "";
            {%- endif %}

        {%- endfor %}
        }
    }
    return 0;
}

static
int ufr_enc_ros2_put_f32(link_t* link, const float val[], int nitems) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
        {% for field in fields -%}

            {% if field.type.type_id in [10,11] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id in [3,5,7,9, 2,4,6,8] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id == 17 %}
            case {{field.index}} : enc_obj->message.{{field.name}} = "";
            {%- endif %}

        {%- endfor %}
        }
    }
    return 0;
}

static
int ufr_enc_ros2_put_u64(link_t* link, const uint64_t val[], int nitems) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
        {% for field in fields -%}

            {% if field.type.type_id in [10,11] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id in [3,5,7,9, 2,4,6,8] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id == 17 %}
            case {{field.index}} : enc_obj->message.{{field.name}} = "";
            {%- endif %}

        {%- endfor %}
        }
    }
    return 0;
}

static
int ufr_enc_ros2_put_i64(link_t* link, const int64_t val[], int nitems) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
        {% for field in fields -%}

            {% if field.type.type_id in [10,11] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id in [3,5,7,9, 2,4,6,8] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id == 17 %}
            case {{field.index}} : enc_obj->message.{{field.name}} = "";
            {%- endif %}

        {%- endfor %}
        }
    }
    return 0;
}

static
int ufr_enc_ros2_put_f64(link_t* link, const double val[], int nitems) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
        {% for field in fields -%}

            {% if field.type.type_id in [10,11] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id in [3,5,7,9, 2,4,6,8] %}
            case {{field.index}} : enc_obj->message.{{field.name}} = val[0];
            {% elif field.type.type_id == 17 %}
            case {{field.index}} : enc_obj->message.{{field.name}} = "";
            {%- endif %}

        {%- endfor %}
        }
    }
    return 0;
}


static
int ufr_enc_ros2_put_str(link_t* link, const char* val) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {
        {% for field in fields -%}

        {% if field.type.type_id == 11 %}
        enc_obj->message.{{field.name}} = val[0];
        {%- endif %}

        {%- endfor %}
    }
    return 0;
}

static
int ufr_enc_ros2_cmd_send(link_t* link) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    enc_obj->publisher->publish(enc_obj->message);
    return UFR_OK;
}

static
int ufr_enc_ros2_cmd_eof(link_t* link) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    enc_obj->publisher->publish(enc_obj->message);
    return UFR_OK;
}

static
int ufr_enc_ros2_cmd_clear(link_t* link) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    {% for field in fields -%}

    {% if field.type.type_id in [10,11] %}
    enc_obj->message.{{field.name}} = 0.0;
    {% elif field.type.type_id in [3,5,7,9, 2,4,6,8] %}
    enc_obj->message.{{field.name}} = 0;
    {% elif field.type.type_id == 17 %}
    enc_obj->message.{{field.name}}.clear();
    {%- endif %}

    {%- endfor %}
    return UFR_OK;
}

static
int ufr_enc_ros2_cmd_seek_str(link_t* link, const char* name) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    {% for field in fields -%}

        {% if field.type.type_id == 11 %}
    if ( strcmp(name, "{{field.name}}") == 0 ){
        enc_obj->index = {{field.index}};
        return UFR_OK;
    }
        {%- endif %}

        
    {%- endfor %}

    // Error
    return -1;
}

static
int ufr_enc_ros2_cmd_next(link_t* link) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    enc_obj->index += 1;
    return 0;
}

static
int ufr_enc_ros2_cmd_enter(link_t* link) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    enc_obj.index1 = 0;
    return 0;
}

static
int ufr_enc_ros2_cmd_leave(link_t* link) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    
    return 0;
}

static
ufr_enc_api_t ufr_enc_ros2_api = {
    .init = ufr_enc_ros2_init,
    .free = ufr_enc_ros2_free,

    .put_u32 = ufr_enc_ros2_put_u32,
    .put_i32 = ufr_enc_ros2_put_i32,
    .put_f32 = ufr_enc_ros2_put_f32,

    .put_u64 = ufr_enc_ros2_put_u64,
    .put_i64 = ufr_enc_ros2_put_i64,
    .put_f64 = ufr_enc_ros2_put_f64,

    .put_str = ufr_enc_ros2_put_str,
    .put_raw = NULL,
    .put_bin = NULL,

    .cmd_enter = ufr_enc_ros2_cmd_enter,
    .cmd_leave = ufr_enc_ros2_cmd_leave,
    .cmd_next = ufr_enc_ros2_cmd_next,
    .cmd_clear = ufr_enc_ros2_cmd_clear,
    .cmd_send = ufr_enc_ros2_cmd_send,
    .cmd_eof = ufr_enc_ros2_cmd_eof,

    .cmd_seek_str = ufr_enc_ros2_cmd_seek_str
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_enc_ros2_new_{{short_name}}(link_t* link, const int type) {
    link->enc_api = &ufr_enc_ros2_api;
    return 0;
}