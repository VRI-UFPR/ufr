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
#include <ros/ros.h>
#include <ros/topic.h>

#include "ufr_gtw_ros_melodic.hpp"

int g_ros_count = 0;

Gateway* g_gtw = NULL;

// ======================================================================================
//  Private
// ======================================================================================


const char* get_last_component(const char* path) {
    // Encontra a última barra '/' no caminho
    const char* last_slash = strrchr(path, '/');

    // Se não houver barra, o caminho já é o nome do arquivo/diretório
    if (last_slash == NULL) {
        return path;
    }

    // Retorna a substring após a última barra
    return &last_slash[1];
}

// ======================================================================================
//  Topic
// ======================================================================================

static
int ufr_ros_loop_cb(void) {
    const bool res = ros::ok();
    return (res) ? UFR_OK : 1;
}

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
        const char* command = getenv("_");
        const char* command_name = get_last_component(command);
        int argc = 1;
        const char* argv[] = {command};
        ros::init(argc, (char**) argv, command_name);
        ufr_loop_put_callback( ufr_ros_loop_cb );
        ufr_log(link, "ROS Node (%s) initialized", command_name);
    }

    g_ros_count += 1;
    
    if ( g_gtw == NULL ) {
        g_gtw = new Gateway();
    }
    link->gtw_obj = g_gtw;

    // success
    return UFR_OK;
}

int ufr_ros_topic_start(link_t* link, int type, const ufr_args_t* args) {
    return UFR_OK;
}

void ufr_ros_topic_stop(link_t* link, int type) {
    
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
    if ( link->dcr_api && link->dcr_api->recv_cb ) {
        return link->dcr_api->recv_cb(link, NULL, 0);
    }
    return -1;
}

static
int ufr_ros_topic_recv_async(link_t* link) {
    if ( link->dcr_api && link->dcr_api->recv_cb ) {
        return link->dcr_api->recv_async_cb(link, NULL, 0);
    }
    return -1;
}

static
ufr_gtw_api_t ufr_ros_melodic_topic_drv = {
    .name = "ROS/Melodic:Topic",
    .type = ufr_ros_topic_type,
    .state = ufr_ros_topic_state,
    .size = ufr_ros_topic_size,
    .boot = ufr_ros_topic_boot,
    .start = ufr_ros_topic_start,
    .stop = ufr_ros_topic_stop,
    .copy = NULL,
    .read = ufr_ros_topic_read,
    .write = ufr_ros_topic_write,
    .recv = ufr_ros_topic_recv,
    .recv_async = ufr_ros_topic_recv_async,
};

// ======================================================================================
//  Socket
// ======================================================================================

struct Decoder {
    ros::master::V_TopicInfo topics;
    ros::master::V_TopicInfo::iterator it;
};


static
int ufr_dcr_ros_humble_boot(link_t* link, const ufr_args_t* args) {
    Decoder* dcr = new Decoder();
    ros::master::getTopics(dcr->topics);
    dcr->it = dcr->topics.begin();

    link->dcr_obj = dcr;
    return UFR_OK;
}

static
int ufr_dcr_ros_humble_get_str(link_t* link, char* val, int size) {
    return 0;
}

static 
int ufr_dcr_ros_humble_recv_cb (link_t* link, char* msg_data, size_t msg_size) {
    return 0;
}


static
ufr_dcr_api_t ufr_dcr_ros_driver = {
    .boot = NULL,
    .close = NULL,

    .recv_cb = ufr_dcr_ros_humble_recv_cb,
    .recv_async_cb = NULL,

    .next = NULL,

    .get_type = NULL,
    .get_nbytes = NULL,
    .get_nitems = NULL,
    .get_rawptr = NULL,

    .get_raw = NULL,
    .get_str = ufr_dcr_ros_humble_get_str,

    .get_u32 = NULL,
    .get_i32 = NULL,
    .get_f32 = NULL,

    .get_u64 = NULL,
    .get_i64 = NULL,
    .get_f64 = NULL,

    .enter = NULL,
    .leave = NULL
};

int ufr_ros_socket_start(link_t* link, int type, const ufr_args_t* args) {
    if ( type == UFR_START_CLIENT ) {
        link->dcr_api = &ufr_dcr_ros_driver;
    }

    return UFR_OK;
}

static
ufr_gtw_api_t ufr_ros_melodic_socket_drv = {
    .name = "ROS/Melodic:Socket",
    .type = ufr_ros_topic_type,
    .state = ufr_ros_topic_state,
    .size = ufr_ros_topic_size,
    .boot = ufr_ros_topic_boot,
    .start = ufr_ros_socket_start,
    .stop = ufr_ros_topic_stop,
    .copy = NULL,
    .read = ufr_ros_topic_read,
    .write = ufr_ros_topic_write,
    .recv = ufr_ros_topic_recv,
    .recv_async = ufr_ros_topic_recv_async,
};

// ======================================================================================
//  Public Constructors
// ======================================================================================

extern "C" {

int ufr_gtw_ros_melodic_new_topic(link_t* out, int type) {
    ufr_link_init(out, &ufr_ros_melodic_topic_drv);
    return UFR_OK;
}

int ufr_gtw_ros_melodic_new_socket(link_t* out, int type) {
    ufr_link_init(out, &ufr_ros_melodic_socket_drv);
    return UFR_OK;
}

int ufr_gtw_ros_melodic_new(link_t* out, int type) {
    return ufr_gtw_ros_melodic_new_topic(out, type);
}

}
