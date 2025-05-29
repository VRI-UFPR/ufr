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
#include <string>
#include <ufr.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "ufr_gtw_ros2.hpp"

struct ll_encoder_t {
    tf2_ros::TransformBroadcaster* tf_broadcaster;
    geometry_msgs::msg::TransformStamped message;
    double th;
    int index;
    std::string frame_id;
    std::string child_frame_id;

    ll_encoder_t() : th{0.0}, index{0} {

    }
};

// ============================================================================
//  String Message Driver
// ============================================================================

static
int ufr_enc_ros2_tf_boot(link_t* link, const ufr_args_t* args) {
    ufr_log(link, "Inicializing TF message");
    
    //
    ll_encoder_t* enc_obj = new ll_encoder_t();
    char buffer[UFR_ARGS_TOKEN];
    enc_obj->frame_id = ufr_args_gets(args, buffer, "@frame", "frame");
    enc_obj->child_frame_id = ufr_args_gets(args, buffer, "@child", "child");

    //
    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;
    enc_obj->tf_broadcaster = new tf2_ros::TransformBroadcaster(gtw_obj->m_node);
    link->enc_obj = enc_obj;

    // success
    ufr_log(link, "Inicialized TF message");
    return UFR_OK;
}

static
void ufr_enc_ros2_tf_close(link_t* link) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    delete(enc_obj->tf_broadcaster);
    delete(enc_obj);
    link->enc_obj = NULL;
}

static
int ufr_enc_ros2_tf_put_u32(link_t* link, const uint32_t* val, int nitems) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {
        
    }
    return 0;
}

static
int ufr_enc_ros2_tf_put_i32(link_t* link, const int32_t* val, int nitems) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {

    }
    return 0;
}

static
int ufr_enc_ros2_tf_put_f32(link_t* link, const float* val, int nitems) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {
        switch(enc_obj->index) {
        case 0:
            enc_obj->message.transform.translation.x = val[0];
            break;
        case 1:
            enc_obj->message.transform.translation.y = val[0];
            break;
        case 2:
            enc_obj->th = val[0];
            break;
        }
        enc_obj->index += 1;
    }
    return 0;
}

static
int ufr_enc_ros2_tf_put_str(link_t* link, const char* val) {
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;
    if ( enc_obj ) {
        
    }
    return 0;
}

static
int ufr_enc_ros2_tf_put_cmd(link_t* link, char cmd) {
    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;
    ll_encoder_t* enc_obj = (ll_encoder_t*) link->enc_obj;

    if ( cmd == '\n' ) {
        // set zero on translation.z
        enc_obj->message.transform.translation.z = 0.0;

        // set transform.rotation
        enc_obj->message.transform.rotation.x = 0.0;
        enc_obj->message.transform.rotation.y = 0.0;
        enc_obj->message.transform.rotation.z = sin(enc_obj->th / 2.0);
        enc_obj->message.transform.rotation.w = cos(enc_obj->th / 2.0);

        // set header
        enc_obj->message.header.stamp = gtw_obj->m_node->now();
        enc_obj->message.header.frame_id = enc_obj->frame_id;
        enc_obj->message.child_frame_id = enc_obj->child_frame_id;

        // send TF message
        enc_obj->tf_broadcaster->sendTransform(enc_obj->message);
        // enc_obj->message.data = "";

        // clear message
        enc_obj->th = 0.0;
        enc_obj->message.transform.translation.x = 0.0;
        enc_obj->message.transform.translation.y = 0.0;
        enc_obj->index = 0;
    }
    return 0;
}

static
ufr_enc_api_t ufr_enc_ros2_tf = {
    .boot = ufr_enc_ros2_tf_boot,
    .close = ufr_enc_ros2_tf_close,
    .clear = NULL,

    .put_u32 = ufr_enc_ros2_tf_put_u32,
    .put_i32 = ufr_enc_ros2_tf_put_i32,
    .put_f32 = ufr_enc_ros2_tf_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_cmd = ufr_enc_ros2_tf_put_cmd,
    .put_str = ufr_enc_ros2_tf_put_str,
    .put_raw = NULL,

    .enter = NULL,
    .leave = NULL,
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_enc_ros2_new_tf(link_t* link, const int type) {
    link->enc_api = &ufr_enc_ros2_tf;
    return 0;
}