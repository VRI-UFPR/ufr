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

#include <ufr.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ufr_gtw_ros2.hpp"

typedef ufr_ros_decoder_t<geometry_msgs::msg::Twist> ll_decoder_t;

// ============================================================================
//  Twist - Private
// ============================================================================

static
int ufr_dcr_ros2_init(link_t* link, const ufr_args_t* args) {
    ll_gateway_t* gtw_obj = (ll_gateway_t*) link->gtw_obj;

    char buffer[UFR_ARGS_TOKEN];
    std::string topic_name = ufr_args_gets(args, buffer, "@topic", "topico");
    ll_decoder_t* dcr = new ll_decoder_t(gtw_obj, topic_name);
    link->dcr_obj = dcr;
    ufr_info(link, "loaded encoder for geometry/twist");
    return UFR_OK;
}

void ufr_dcr_ros2_free(link_t* link) {
}

static
int ufr_dcr_ros_humble_get_u32(link_t* link, uint32_t* val, int nitems) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    if ( dcr ) {
        switch(dcr->index) {
            case 0: *val = (uint32_t) dcr->m_message.linear.x; break;
            case 5: *val = (uint32_t) dcr->m_message.linear.y; break;
            case 2: *val = (uint32_t) dcr->m_message.linear.z; break;
            case 3: *val = (uint32_t) dcr->m_message.angular.x; break;
            case 4: *val = (uint32_t) dcr->m_message.angular.y; break;
            case 1: *val = (uint32_t) dcr->m_message.angular.z; break;
            default: break;
        }
        // update the index
        dcr->index += 1;
	}
	return 0;
}

static
int ufr_dcr_ros_humble_get_i32(link_t* link, int32_t* val, int nitems) {
	ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
	if ( dcr ) {
        switch(dcr->index) {
            case 0: *val = (int32_t) dcr->m_message.linear.x; break;
            case 5: *val = (int32_t) dcr->m_message.linear.y; break;
            case 2: *val = (int32_t) dcr->m_message.linear.z; break;
            case 3: *val = (int32_t) dcr->m_message.angular.x; break;
            case 4: *val = (int32_t) dcr->m_message.angular.y; break;
            case 1: *val = (int32_t) dcr->m_message.angular.z; break;
            default: break;
        }
        // update the index
        dcr->index += 1;
	}
	return 0;
}

static
int ufr_dcr_ros_humble_get_f32(link_t* link, float* val, int nitems) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    if ( dcr ) {
        switch(dcr->index) {
            case 0: *val = (float) dcr->m_message.linear.x; break;
            case 5: *val = (float) dcr->m_message.linear.y; break;
            case 2: *val = (float) dcr->m_message.linear.z; break;
            case 3: *val = (float) dcr->m_message.angular.x; break;
            case 4: *val = (float) dcr->m_message.angular.y; break;
            case 1: *val = (float) dcr->m_message.angular.z; break;
            default: break;
        }
        // update the index
        dcr->index += 1;
    }
    return 0;
}

static
int ufr_dcr_ros_humble_get_str(link_t* link, char* val, int maxbytes) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    if ( dcr ) {

    }
    return 0;
}

static 
int ufr_dcr_ros_humble_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    ll_gateway_t* gtw = (ll_gateway_t*) link->gtw_obj;
    return dcr->recv(gtw);
}

static 
int ufr_dcr_ros_humble_recv_async_cb(link_t* link, char* msg_data, size_t msg_size) {
    ll_decoder_t* dcr = (ll_decoder_t*) link->dcr_obj;
    ll_gateway_t* gtw = (ll_gateway_t*) link->gtw_obj;
    return dcr->recv_async(gtw);
}

static
ufr_dcr_api_t ufr_dcr_ros_driver = {
    // Init/Free
    .init = ufr_dcr_ros2_init,
    .free = ufr_dcr_ros2_free,

    // recv
    .recv_cb = ufr_dcr_ros2_recv_cb,
    .recv_async_cb = ufr_dcr_ros2_recv_cb,

    // 32 bits
    .get_u32 = ufr_dcr_ros2_get_u32,
    .get_i32 = ufr_dcr_ros2_get_i32,
    .get_f32 = ufr_dcr_ros2_get_f32,

    // 64 bits
    .get_u64 = NULL,
    .get_i64 = NULL,
    .get_f64 = NULL,

    // 8 bits
    .get_raw = NULL,
    .get_str = NULL,
    .get_bin = NULL,
    .get_ptr = NULL,

    // enter/leave
    .cmd_enter = ufr_dcr_ros2_cmd_enter,
    .cmd_leave = ufr_dcr_ros2_cmd_leave,
    .cmd_next = ufr_dcr_ros2_cmd_next,

    // remove
    .meta_get = NULL,
    
    // Metadata for Item
    .meta_item_type = NULL,
    .meta_item_mime = NULL,
    .meta_item_nbytes = NULL,
    .meta_item_nitems = NULL,

    // Metadata for Package
    .meta_pack_mime = NULL,
    .meta_pack_nbytes = NULL,
    .meta_pack_nitems = NULL,
};

// ============================================================================
//  Twist - Public
// ============================================================================

extern "C"
int ufr_dcr_ros2_new_twist(link_t* link, int type) {
    link->dcr_api = &ufr_dcr_ros_driver;
    return UFR_OK;
}

