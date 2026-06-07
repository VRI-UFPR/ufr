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
#include <string.h>
#include <webots/device.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <ufr.h>

#include "ufr_webots.h"

typedef struct {
    WbDeviceTag left;
    WbDeviceTag right;
    double wheel_left, wheel_right;
    double wheel_left_last, wheel_right_last;
    double x,y,th;
    bool is_first_read;
    uint8_t index;
} dcr_encoders_t;

// ============================================================================
//  Decoder
// ============================================================================

static
int ufr_dcr_pose_init(link_t* link, const ufr_args_t* args) {
    // new decoder
    dcr_encoders_t* dcr = malloc(sizeof(dcr_encoders_t));
    dcr->index = 0;
    dcr->is_first_read = true;
    dcr->x = 0.0;
    dcr->y = 0.0;
    dcr->th = 0.0;


    // Search by position sensors
    char const* left_name = NULL;
    char const* right_name = NULL;
    const int n_devices = wb_robot_get_number_of_devices();
    for(int i=0; i<n_devices; i++) {
        WbDeviceTag tag = wb_robot_get_device_by_index(i);
        const char *name = wb_device_get_name(tag);
        WbNodeType type = wb_device_get_node_type(tag);
        if ( type == WB_NODE_POSITION_SENSOR ) {
            if ( strstr(name, "left") != NULL ) {
                left_name = name;
            } else if ( strstr(name, "right") != NULL ) {
                right_name = name;
            }
        }
    }

    if ( left_name == NULL || right_name == NULL ) {
        return ufr_error(link, 1, "Not found left or right positional sensor");
    }

    // get sensor for both wheels
    ufr_log_ini(link, "Inicializando o par de sensores: (%s, %s)", left_name, right_name);
    dcr->left = wb_robot_get_device(left_name);
    dcr->right = wb_robot_get_device(right_name);

    // enable the encoders
    const int time_step = ufr_gtw_webots_get_time_step();
    wb_position_sensor_enable(dcr->left, time_step);
    wb_position_sensor_enable(dcr->right, time_step);
    

    // success
    link->dcr_obj = dcr;
    return UFR_OK;
}

static
void ufr_dcr_pose_free(link_t* link) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    if ( dcr ) {
        free(dcr);
        link->dcr_obj = NULL;
    }
}

static
int ufr_dcr_pose_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    if ( dcr ) {
        // get new read
        dcr->wheel_left = wb_position_sensor_get_value(dcr->left);
        dcr->wheel_right = wb_position_sensor_get_value(dcr->right);
        dcr->index = 0;

        // case is not first read
        if ( dcr->is_first_read == false ) {
            const double diff_left = dcr->wheel_left - dcr->wheel_left_last;
            const double diff_right = dcr->wheel_right - dcr->wheel_right_last;
            dcr->x += ((diff_left + diff_right) * cos(dcr->th)) / 100.0;
            dcr->y += ((diff_left + diff_right) * sin(dcr->th)) / 100.0;
            dcr->th += (diff_left - diff_right) * 0.3;
        }

        // update last read
        dcr->wheel_left_last = dcr->wheel_left;
        dcr->wheel_right_last = dcr->wheel_right;
        dcr->is_first_read = false;
    } else {
        ufr_fatal(link, 1, "dcr is not booted");
    }
    return UFR_OK;
}

static
int ufr_dcr_pose_get_u32(link_t* link, uint32_t* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        switch (dcr->index) {
            case 0: val[wrote] = dcr->x; break;
            case 1: val[wrote] = dcr->y; break;
            case 2: val[wrote] = dcr->th; break;
            default: val[wrote] = 0; break;
        }
        dcr->index += 1;
    }
    return wrote;
}

static
int ufr_dcr_pose_get_i32(link_t* link, int32_t* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        switch (dcr->index) {
            case 0: val[wrote] = dcr->x; break;
            case 1: val[wrote] = dcr->y; break;
            case 2: val[wrote] = dcr->th; break;
            default: val[wrote] = 0; break;
        }
        dcr->index += 1;
    }
    return wrote;
}

static
int ufr_dcr_pose_get_f32(link_t* link, float* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    if ( dcr ) {
        for (; wrote<nitems; wrote++) {
            switch (dcr->index) {
                case 0: val[wrote] = dcr->x; break;
                case 1: val[wrote] = dcr->y; break;
                case 2: val[wrote] = dcr->th; break;
                default: val[wrote] = 0; break;
            }
            dcr->index += 1;
        }
    } else {
        return ufr_error(link, 1, "Decoder is null");
    }
    return wrote;
}

static
int ufr_dcr_pose_get_u64(link_t* link, uint64_t* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        switch (dcr->index) {
            case 0: val[wrote] = dcr->x; break;
            case 1: val[wrote] = dcr->y; break;
            case 2: val[wrote] = dcr->th; break;
            default: val[wrote] = 0; break;
        }
        dcr->index += 1;
    }
    return wrote;
}

static
int ufr_dcr_pose_get_i64(link_t* link, int64_t* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        switch (dcr->index) {
            case 0: val[wrote] = dcr->x; break;
            case 1: val[wrote] = dcr->y; break;
            case 2: val[wrote] = dcr->th; break;
            default: val[wrote] = 0; break;
        }
        dcr->index += 1;
    }
    return wrote;
}

static
int ufr_dcr_pose_get_f64(link_t* link, double* val, int nitems) {
    dcr_encoders_t* dcr = (dcr_encoders_t*) link->dcr_obj;
    int wrote = 0;
    if ( dcr ) {
        for (; wrote<nitems; wrote++) {
            switch (dcr->index) {
                case 0: val[wrote] = dcr->x; break;
                case 1: val[wrote] = dcr->y; break;
                case 2: val[wrote] = dcr->th; break;
                default: val[wrote] = 0; break;
            }
            dcr->index += 1;
        }
    } else {
        return ufr_error(link, 1, "Decoder is null");
    }
    return wrote;
}

static
int ufr_dcr_pose_get_str(link_t* link, char* ret_val, int size) {
    return UFR_OK;
}

static
int ufr_dcr_pose_enter(link_t* link) {
    return -1;
}

static
int ufr_dcr_pose_leave(link_t* link) {
    return -1;
}

static
ufr_dcr_api_t dcr_pose_api = {
    .init = ufr_dcr_pose_init,
    .free = ufr_dcr_pose_free,

    // Receive
    .recv_cb = ufr_dcr_pose_recv_cb,
    .recv_async_cb = ufr_dcr_pose_recv_cb,

    // 32 bits
    .get_u32 = ufr_dcr_pose_get_u32,
    .get_i32 = ufr_dcr_pose_get_i32,
    .get_f32 = ufr_dcr_pose_get_f32,

    // 64 bits
    .get_u64 = ufr_dcr_pose_get_u64,
    .get_i64 = ufr_dcr_pose_get_i64,
    .get_f64 = ufr_dcr_pose_get_f64,

    // Binary and String
    .get_raw = NULL,
    .get_str = ufr_dcr_pose_get_str,

    // Meta Item
    .meta_item_type = NULL,
    .meta_item_mime = NULL,
    .meta_item_nbytes = NULL,
    .meta_item_nitems = NULL,

    // Meta Package
    .meta_pack_mime = NULL,
    .meta_pack_nbytes = NULL,
    .meta_pack_nitems = NULL,

    // Commands
    .cmd_enter = ufr_dcr_pose_enter,
    .cmd_leave = ufr_dcr_pose_leave,
    .cmd_next = NULL
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_dcr_webots_new_pose(link_t* link, int type) {
    link->dcr_api = &dcr_pose_api;
    return UFR_OK;
}