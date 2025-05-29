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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/device.h>
#include <signal.h>

#include "ufr_webots.h"

int g_time_step = 100;

// ============================================================================
//  Loop callback
// ============================================================================

static
int webots_loop_cb(void) {
    wb_robot_step(g_time_step);
    return UFR_OK;
}

void webots_interrupt_handler(int dummy) {
    ufr_loop_set_end();
}

// ============================================================================
//  Gateway
// ============================================================================

static
int    ufr_gtw_webots_type(const link_t* link) {
    return 0;
}

static
int    ufr_gtw_webots_state(const link_t* link) {
    return 0;
}

static
size_t ufr_gtw_webots_size(const link_t* link, int type) {
    return 0;
}

static
int  ufr_gtw_webots_boot(link_t* link, const ufr_args_t* args) {
    // Library initialization
    static uint8_t is_initialized = 0;
    if ( is_initialized == 0 ) {
        ufr_log(link, "inicializado WeBots");
        wb_robot_init();
        ufr_loop_put_callback( webots_loop_cb );
        signal(SIGINT, webots_interrupt_handler);

        const char* model = wb_robot_get_model();
        ufr_log(link, "Robot model: %s", model);

        const int n_devices = wb_robot_get_number_of_devices();
        for(int i=0; i<n_devices; i++) {
            WbDeviceTag tag = wb_robot_get_device_by_index(i);
            const char *name = wb_device_get_name(tag);
            WbNodeType type = wb_device_get_node_type(tag);

            // do something with the device
            ufr_info(link, "Device #%d, name = \"%s\", type = %d", i, name, type);
        }

    }
    is_initialized = 1;

    // Select the encoder ou decoder
    char buffer[UFR_ARGS_TOKEN];
    const char* dev_type = ufr_args_gets(args, buffer, "@topic", "");
 
    // cmd_vel
    if ( strcmp(dev_type, "cmd_vel") == 0 || strcmp(dev_type, "/cmd_vel") == 0 ) {
        ufr_log(link, "Carregado encoder para cmd_vel");
        ufr_enc_webots_new_motors(link, UFR_START_PUBLISHER);
        ufr_boot_enc(link, args);

    } else if ( strcmp(dev_type, "cmd_vel_3d") == 0 || strcmp(dev_type, "/cmd_vel_3d") == 0 ) {
        ufr_log(link, "Carregado encoder para cmd_vel");
        ufr_enc_webots_new_motors_drone(link, UFR_START_PUBLISHER);
        ufr_boot_enc(link, args);

    // Encoders da roda
    } else if ( strcmp(dev_type, "encoders") == 0 ) {
        ufr_dcr_webots_new_encoders(link, UFR_START_SUBSCRIBER);
        ufr_boot_dcr(link, args);

    // Lidar
    } else if ( strcmp(dev_type, "scan") == 0 || strcmp(dev_type, "/scan") == 0 ) {
        ufr_dcr_webots_new_lidar(link, UFR_START_SUBSCRIBER);
        ufr_boot_dcr(link, args);

    // Posicao
    } else if ( strcmp(dev_type, "pose") == 0 || strcmp(dev_type, "/pose") == 0 ) {
        ufr_dcr_webots_new_pose(link, UFR_START_SUBSCRIBER);
        ufr_boot_dcr(link, args);

    // Error
    } else {
        return ufr_error(link, -1, "Device not avaliable");
    }

    return UFR_OK;
}

static
int  ufr_gtw_webots_start(link_t* link, int type, const ufr_args_t* args) {
    return UFR_OK;
}

static
void ufr_gtw_webots_stop(link_t* link, int type) {

}

static
int  ufr_gtw_webots_copy(link_t* link, link_t* out) {

}

static
size_t ufr_gtw_webots_read(link_t* link, char* buffer, size_t length) {
    return 0;
}

static
size_t ufr_gtw_webots_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

static
int ufr_gtw_webots_recv(link_t* link) {
    if ( link->dcr_api != NULL  && link->dcr_api->recv_cb != NULL ) {
        link->dcr_api->recv_cb(link, NULL, 0);
        return UFR_OK;
    }
    return UFR_OK;
}

static
int ufr_gtw_webots_recv_async(link_t* link) {
    if ( link->dcr_api != NULL  && link->dcr_api->recv_async_cb != NULL ) {
        link->dcr_api->recv_async_cb(link, NULL, 0);
        return UFR_OK;
    }
    return UFR_OK;
}

static
int ufr_gtw_webots_send(link_t* link) {
    return UFR_OK;
}

static
int ufr_gtw_webots_accept(link_t* link, link_t* out_client) {
    return UFR_OK;
}

static
const char* ufr_gtw_webots_test_args(const link_t* link) {
    return "";
}

static
ufr_gtw_api_t ufr_gtw_webots_api = {
    .name = "webots",
	.type = ufr_gtw_webots_type,
	.state = ufr_gtw_webots_state,
	.size = ufr_gtw_webots_size,

	.boot = ufr_gtw_webots_boot,
	.start = ufr_gtw_webots_start,
	.stop = ufr_gtw_webots_stop,
	.copy = ufr_gtw_webots_copy,

	.read = ufr_gtw_webots_read,
	.write = ufr_gtw_webots_write,

	.recv = ufr_gtw_webots_recv,
	.recv_async = ufr_gtw_webots_recv_async,

    .accept = ufr_gtw_webots_accept,

    .test_args = ufr_gtw_webots_test_args,
};


// ============================================================================
//  Public Function
// ============================================================================

int ufr_gtw_webots_new(link_t* link, int type, const ufr_args_t* args) {
    ufr_link_init(link, &ufr_gtw_webots_api);
    return UFR_OK;
}

int ufr_gtw_webots_get_time_step() {
    return g_time_step;
}

/**

video = ufr_subscriber("video")
const char* type = ufr_meta_str(&video, "type") -> "r:image:jpg", "r:text:json", "r:text:xml", "r:image:png", "r:image:CV_8UC3"
int cols = ufr_meta_i32(&video, "cols");
int rows = ufr_meta_i32(&video, "rows");


    WB_NODE_ACCELEROMETER,
  WB_NODE_ALTIMETER,
  WB_NODE_BRAKE,
  WB_NODE_CAMERA,
  WB_NODE_COMPASS,
  WB_NODE_CONNECTOR,
  WB_NODE_DISPLAY,
  WB_NODE_DISTANCE_SENSOR,
  WB_NODE_EMITTER,
  WB_NODE_GPS,
  WB_NODE_GYRO,
  WB_NODE_INERTIAL_UNIT,
  WB_NODE_LED,
  WB_NODE_LIDAR,
  WB_NODE_LIGHT_SENSOR,
  WB_NODE_LINEAR_MOTOR,
  WB_NODE_PEN,
  WB_NODE_POSITION_SENSOR,
  WB_NODE_PROPELLER,
  WB_NODE_RADAR,
  WB_NODE_RANGE_FINDER,
  WB_NODE_RECEIVER,
  WB_NODE_ROTATIONAL_MOTOR,
  WB_NODE_SKIN,
  WB_NODE_SPEAKER,
  WB_NODE_TOUCH_SENSOR,
  WB_NODE_VACUUM_GRIPPER,

  https://itooktheredpill.irgendwo.org/2020/rooting-xiaomi-vacuum-robot/
  stytj02ym

  Mjst1s

  xiaomi s20+


 */