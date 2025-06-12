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

// ======================================================================================
//  Header
// ======================================================================================

#include <memory>
#include <ufr.h>

#include "ufr_gtw_ros2.hpp"

size_t g_ros_count = 0;
ll_gateway_t* g_gateway = NULL;

// ======================================================================================
//  Callback for Main Loop
// ======================================================================================

static
int ufr_ros_humble_loop_cb(void) {
    return ( rclcpp::ok() ) ? UFR_OK : 1;
}

// ======================================================================================
//  Subscribe
// ======================================================================================

int ufr_ros_topic_type(const link_t* link) {
    return 0;
}

int ufr_ros_topic_state(const link_t* link) {
    return 0;
}

size_t ufr_ros_topic_size(const link_t* link, int type) {
    return 0;
}

int ufr_ros_topic_boot(link_t* link, const ufr_args_t* args) {
    if ( g_ros_count == 0 ) {
        ufr_info(link, "ROS2 start");
        const char* command = getenv("_");
        const char* argv[] = {command, NULL};
        rclcpp::init(1, argv);
        g_gateway = new ll_gateway_t();
        ufr_loop_put_callback( ufr_ros_humble_loop_cb );
        ufr_info(link, "ROS2 initalized");
    }
    g_ros_count += 1;
    link->gtw_obj = (void*) g_gateway;
    return UFR_OK;
}

int ufr_ros_topic_start(link_t* link, int type, const ufr_args_t* args) {
    return UFR_OK;
}

void ufr_ros_topic_stop(link_t* link, int type) {
    if ( g_ros_count <= 1 ) {
        ufr_info(link, "ROS2 shutdown");
        rclcpp::shutdown();
        g_ros_count = 0;
    } else {
        g_ros_count -= 1;
    }
}

static
size_t ufr_ros_topic_read(link_t* link, char* buffer, size_t length) {
	return 0;
}

static
size_t ufr_ros_topic_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

static
int ufr_ros_topic_recv(link_t* link) {
    if ( link && link->dcr_api && link->dcr_api->recv_cb ) {
        return link->dcr_api->recv_cb(link, NULL, 0U);
    }
    return -1;
}

static
int ufr_ros_topic_recv_async(link_t* link) {
    if ( link->dcr_api->recv_async_cb == NULL ) {
        ufr_log(link, "ERROR123");
        return -1;
    }
    return link->dcr_api->recv_async_cb(link, NULL, 0U);
}

ufr_gtw_api_t ufr_ros_humble_topic_drv = {
    .name = "ROS:Topic",
    .type = ufr_ros_topic_type,
    .state = ufr_ros_topic_state,
    .size = ufr_ros_topic_size,
    .boot = ufr_ros_topic_boot,
    .start = ufr_ros_topic_start,
    .stop = ufr_ros_topic_stop,
    .read = ufr_ros_topic_read,
    .write = ufr_ros_topic_write,
    .recv = ufr_ros_topic_recv,
    .recv_async = ufr_ros_topic_recv_async,
};

// ======================================================================================
//  Constructors
// ======================================================================================

extern "C"
int ufr_gtw_ros2_new_topic(link_t* out, int type) {
    ufr_link_init(out, &ufr_ros_humble_topic_drv);
    return UFR_OK;
}

extern "C"
int ufr_gtw_ros2_new(link_t* out, int type) {
    if ( type == UFR_START_PUBLISHER) {
        ufr_gtw_ros2_new_topic(out, type);
    } else if ( type == UFR_START_SUBSCRIBER) {
        ufr_gtw_ros2_new_topic(out, type);
    } else {
        return ufr_error(out, 1, "Start type is invalid");
    }
    return UFR_OK;
}