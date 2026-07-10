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
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ufr_gtw_ros2.hpp"


struct ll_encoder {
    tf2_ros::TransformBroadcaster* tf_broadcaster;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
    sensor_msgs::msg::LaserScan message;
    std::string parent_id;
    std::string frame_id;
    int index;
    int index2;

    float def_angle_min;
    float def_angle_max;

    float tf_x, tf_y, tf_z, tf_th;
    geometry_msgs::msg::TransformStamped tf_msg;

    ll_encoder() : index{0}, index2{0} {}
};

extern "C"
int ufr_enc_ros_humble_new_laser_scan(link_t* link, int type);

// ============================================================================
//  LaserScan Encoder
// ============================================================================

static
int ufr_enc_ros2_init(link_t* link, const ufr_args_t* args) {
    char buffer[UFR_ARGS_TOKEN];

    // New Encoder and set Parameters
    ll_encoder* enc_obj = new ll_encoder();
    enc_obj->frame_id = ufr_args_gets(args, buffer, "@frame_id", "laser");
    ufr_log(link, "@frame_id %s", enc_obj->frame_id.c_str());

    // enc_obj->def_angle_max = ufr_args_getf(args, "@angle_max", 0.0);
    // @order angle_max,angle_min,ranges @default {range_min: 0, range_max: 10}

    // Open the publisher
    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;
    const std::string topic_name = ufr_args_gets(args, buffer, "@topic", "/scan");
    ufr_log(link, "@topic %s", topic_name.c_str());
    enc_obj->publisher = gtw_obj->m_node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);

    // Open the TF Broadcast
    enc_obj->parent_id = ufr_args_gets(args, buffer, "@parent_id", "base_footprint");
    ufr_log(link, "@parent_id %s", enc_obj->parent_id.c_str());
    enc_obj->tf_broadcaster = new tf2_ros::TransformBroadcaster(gtw_obj->m_node);

    // Set the TF position
    const float tf_x = ufr_args_getf(args, "@tf_x", 0.0);
    ufr_log(link, "@tf_x %f", tf_x);
    const float tf_y = ufr_args_getf(args, "@tf_y", 0.0);
    ufr_log(link, "@tf_y %f", tf_y);
    const float tf_z = ufr_args_getf(args, "@tf_z", 0.0);
    ufr_log(link, "@tf_z %f", tf_z);
    const float tf_th = ufr_args_getf(args, "@tf_th", 0.0);
    ufr_log(link, "@tf_th %f", tf_th);

    // Set TF Message header
    enc_obj->tf_msg.header.frame_id = enc_obj->parent_id;
    enc_obj->tf_msg.child_frame_id = enc_obj->frame_id;

    // Set on translation
    enc_obj->tf_msg.transform.translation.x = tf_x;
    enc_obj->tf_msg.transform.translation.y = tf_y;
    enc_obj->tf_msg.transform.translation.z = tf_z;

    // Set on transform rotation
    enc_obj->tf_msg.transform.rotation.x = 0;
    enc_obj->tf_msg.transform.rotation.y = 0;
    enc_obj->tf_msg.transform.rotation.z = sin(tf_th / 2.0);
    enc_obj->tf_msg.transform.rotation.w = cos(tf_th / 2.0);

    // Success
    link->enc_obj = enc_obj;
    ufr_info(link, "loaded encoder for sensor_msgs/LaserScan");
    return UFR_OK;
}

static
void ufr_enc_ros2_free(link_t* link) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj != NULL ) {
        delete(enc_obj);
    }
    link->enc_obj = NULL;
}

static
int ufr_enc_ros2_cmd_clear(link_t* link) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    enc_obj->index = 0;
    enc_obj->message.angle_min = 0;
    enc_obj->message.angle_max = 0;
    enc_obj->message.angle_increment = 0;
    enc_obj->message.time_increment = 0;
    enc_obj->message.angle_increment = 0;
    enc_obj->message.scan_time = 0;
    enc_obj->message.range_min = 0;
    enc_obj->message.range_min = 0;
    enc_obj->message.range_max = 0;
    return UFR_OK;
}

static
int ufr_enc_ros2_put_u32(link_t* link, const uint32_t* val, int nitems) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj ) {
        return -1;
    }

    int i=0;
    for (;i<nitems; i++) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.angle_min = val[i]; enc_obj->index += 1;break;
            case 1: enc_obj->message.angle_max = val[i]; enc_obj->index += 1;break;
            case 2: enc_obj->message.angle_increment = val[i]; enc_obj->index += 1;break;
            case 3: enc_obj->message.time_increment = val[i]; enc_obj->index += 1;break;
            case 4: enc_obj->message.scan_time = val[i]; enc_obj->index += 1;break;
            case 5: enc_obj->message.range_min = val[i]; enc_obj->index += 1;break;
            case 6: enc_obj->message.range_max = val[i]; enc_obj->index += 1;break;
            case 7: enc_obj->message.ranges[enc_obj->index2++] = val[i];break;
            case 8: enc_obj->message.intensities[enc_obj->index2++] = val[i];break;
            default: break;
        }
    }
    return i;
}

static
int ufr_enc_ros2_put_i32(link_t* link, const int32_t* val, int nitems) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    for (int i=0; i<nitems; i++) {
        switch(enc_obj->index) {
            case 0: enc_obj->message.angle_min = val[i]; enc_obj->index += 1;break;
            case 1: enc_obj->message.angle_max = val[i]; enc_obj->index += 1;break;
            case 2: enc_obj->message.angle_increment = val[i]; enc_obj->index += 1;break;
            case 3: enc_obj->message.time_increment = val[i]; enc_obj->index += 1;break;
            case 4: enc_obj->message.scan_time = val[i]; enc_obj->index += 1;break;
            case 5: enc_obj->message.range_min = val[i]; enc_obj->index += 1;break;
            case 6: enc_obj->message.range_max = val[i]; enc_obj->index += 1;break;
            case 7: enc_obj->message.ranges[enc_obj->index2++] = val[i];break;
            case 8: enc_obj->message.intensities[enc_obj->index2++] = val[i];break;
            default: break;
        }
    }

    return 0;
}

static
int ufr_enc_ros2_put_f32(link_t* link, const float* val, int nitems) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj == NULL ) {
        return -1;
    }

    // Return count
    int i;

    // array Ranges
    if ( enc_obj->index == 7 ) {
        if ( ( (size_t) enc_obj->index2 + nitems) > enc_obj->message.ranges.size() ) {
            nitems = enc_obj->message.ranges.size() - enc_obj->index2;
        }

        for (i=0; i<nitems; i++) {
            enc_obj->message.ranges[enc_obj->index2++] = val[i];
        }
    
    // array intensities
    } else if ( enc_obj->index == 8 ) {
        if ( ( (size_t) enc_obj->index2 + nitems ) > enc_obj->message.intensities.size() ) {
            nitems = enc_obj->message.intensities.size() - enc_obj->index2;
        }
        for (i=0; i<nitems; i++) {
            enc_obj->message.intensities[enc_obj->index2++] = val[i];
        }

    // others
    } else {
        for (i=0; i<nitems; i++) {
            switch(enc_obj->index) {
                case 0: enc_obj->message.angle_min = val[i]; break;
                case 1: enc_obj->message.angle_max = val[i]; break;
                case 2: enc_obj->message.angle_increment = val[i]; break;
                case 3: enc_obj->message.time_increment = val[i]; break;
                case 4: enc_obj->message.scan_time = val[i]; break;
                case 5: enc_obj->message.range_min = val[i]; break;
                case 6: enc_obj->message.range_max = val[i]; break;
                case 7: enc_obj->message.ranges[0] = val[i]; break;
                case 8: enc_obj->message.intensities[0] = val[i]; break;
                default: break;
            }
            enc_obj->index += 1;
        }
    }

    // success
    return i;
}

static
int ufr_enc_ros2_put_str(link_t* link, const char* val) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj ) {

    }
    return 0;
}

static
int ufr_enc_ros2_cmd_send(link_t* link) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;

    // get the timestamp
    auto stamp = rclcpp::Clock().now();
   
    // send TF message
    enc_obj->tf_msg.header.stamp = stamp;
    enc_obj->tf_broadcaster->sendTransform(enc_obj->tf_msg);

    // send data to Topic
    enc_obj->message.header.stamp = stamp;
    enc_obj->message.header.frame_id = enc_obj->frame_id;
    enc_obj->publisher->publish(enc_obj->message);
    enc_obj->index = 0;

    // Success
    ufr_info(link, "sent message sensors/LaserScan");
    return UFR_OK;
}

static
int ufr_enc_ros2_cmd_enter(struct _link* link, size_t maxsize) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj->index == 7 ) {
        enc_obj->message.ranges.resize(maxsize);
        enc_obj->index2 = 0;
    } else if ( enc_obj->index == 8 ) {
        enc_obj->message.intensities.resize(maxsize);
        enc_obj->index2 = 0;
    } else {
        return ufr_error(link, -1, "Invalid index %d to enter", enc_obj->index);
    }

    return UFR_OK;
}

static
int ufr_enc_ros2_cmd_leave(struct _link* link) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj->index == 7 || enc_obj->index == 8 ) {
        enc_obj->index += 1;
    } else {
        return ufr_error(link, -1, "Invalid index %d to leave", enc_obj->index);
    }
    return UFR_OK;
}

static
int ufr_enc_ros2_cmd_next(struct _link* link) {
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    if ( enc_obj->index > 8 ) {
        return -1;
    }
    enc_obj->index += 1;
    return UFR_OK;
}

static
int ufr_enc_ros2_cmd_seek_str(struct _link* link, const char* name) {
    static const char* names[] = {
        "angle_min", "angle_max", "angle_increment", "time_increment", "scan_time",
        "range_min", "range_max", "ranges", "intensities"
    };

    // busca o indice
    ll_encoder* enc_obj = (ll_encoder*) link->enc_obj;
    for (int i=0; i<9; i++) {
        if ( strcmp(name, names[i]) == 0 ){
            enc_obj->index = i;
            return UFR_OK;
        } 
    }

    // Error, nao encontrou
    return -1;
}


static
ufr_enc_api_t ufr_enc_ros_api = {
    .init = ufr_enc_ros2_init,
    .free = ufr_enc_ros2_free,

    .put_u32 = ufr_enc_ros2_put_u32,
    .put_i32 = ufr_enc_ros2_put_i32,
    .put_f32 = ufr_enc_ros2_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_str = ufr_enc_ros2_put_str,
    .put_raw = NULL,
    .put_bin = NULL,

    .cmd_enter = ufr_enc_ros2_cmd_enter,
    .cmd_leave = ufr_enc_ros2_cmd_leave,
    .cmd_next = ufr_enc_ros2_cmd_next,
    .cmd_clear = ufr_enc_ros2_cmd_clear,
    .cmd_send = ufr_enc_ros2_cmd_send,
    .cmd_eof = ufr_enc_ros2_cmd_send,

    .cmd_seek_str = ufr_enc_ros2_cmd_seek_str
};

// ============================================================================
//  Public
// ============================================================================

extern "C"
int ufr_enc_ros2_new_laser_scan(link_t* link, int type) {
    link->enc_api = &ufr_enc_ros_api;
    return UFR_OK;
}

extern "C"
int ufr_enc_ros2_new_laserscan(link_t* link, int type) {
    link->enc_api = &ufr_enc_ros_api;
    return UFR_OK;
}