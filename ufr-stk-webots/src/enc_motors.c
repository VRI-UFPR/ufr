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
#include <webots/device.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <string.h>

#include <stdio.h>

#define MAX_MOTORS    4

typedef struct {
    uint8_t motors_count;     // 1 motor de cada lado, 2 motores de cada lado
    WbDeviceTag left[MAX_MOTORS];
    WbDeviceTag right[MAX_MOTORS];
    double vel, rotvel;
    uint8_t index;
} enc_motors_t;

// ============================================================================
//  Encoder
// ============================================================================

static
int ufr_enc_motors_init(link_t* link, const ufr_args_t* args) {
    // Inicialize the local variables
    uint8_t left_count = 0;
    uint8_t right_count = 0;
     
    char const* dev_left_default[MAX_MOTORS];
    char const* dev_right_default[MAX_MOTORS];
    for (uint8_t i=0; i<MAX_MOTORS; i++) {
        dev_left_default[i] = NULL;
        dev_right_default[i] = NULL;
    }

    // Search by motors
    const int n_devices = wb_robot_get_number_of_devices();
    for(int i=0; i<n_devices; i++) {
        WbDeviceTag tag = wb_robot_get_device_by_index(i);
        const char *name = wb_device_get_name(tag);
        WbNodeType type = wb_device_get_node_type(tag);
        if ( type == WB_NODE_ROTATIONAL_MOTOR ) {
            if ( strstr(name, "left") != NULL ) {
                dev_left_default[left_count] = name;
                left_count += (left_count<MAX_MOTORS) ? 1 : 0;

            } else if ( strstr(name, "right") != NULL ) {
                dev_right_default[right_count] = name;
                right_count += (right_count<MAX_MOTORS) ? 1 : 0;
            }
        }
    }

    // Check if there are same number of motors each side
    if ( left_count != right_count ) {
        return ufr_error(link, 1, "There are differents number of motors for each side");
    }

    if ( left_count > MAX_MOTORS ) {
        return ufr_error(link, 1, "More than %d motors for each side", MAX_MOTORS);
    }

    // Instance the encoder object
    enc_motors_t* enc = malloc(sizeof(enc_motors_t));

    // prepare the motors
    const uint8_t motors_count = left_count;
    for (uint8_t i=0; i<motors_count; i++) {
        // Open the left motor
        char attr_left_name[16] = "@left\0\0";
        attr_left_name[5] = '0' + i;                   // put '1' or '2'
        char buffer_left[UFR_ARGS_TOKEN];
        const char* dev_left_name = ufr_args_gets(args, buffer_left, attr_left_name, dev_left_default[i]);
        ufr_info(link, "@left%d %s", i, dev_left_name);

        // Open the right motor
        char attr_right_name[16] = "@right\0\0";
        attr_right_name[6] = '0' + i;                   // put '1' or '2'
        char buffer_right[UFR_ARGS_TOKEN];
        const char* dev_right_name = ufr_args_gets(args, buffer_right, attr_right_name, dev_right_default[i]);
        ufr_info(link, "@right%d %s", i, dev_right_name);

        // Verify there are name for the motors
        if ( dev_left_name == NULL ) {
            return ufr_error(link, -1, "Left rotacional motor not found and @left%d not provided", i);
        }
        if ( dev_right_name == NULL ) {
            return ufr_error(link, -1, "Right rotacional motor not found and @right%d not provided", i);
        }

        // Get the device from Webots
        ufr_log_ini(link, "Inicializando o par de motores %d : (%s, %s)", i, dev_left_name, dev_right_name);
        enc->left[i] = wb_robot_get_device( dev_left_name );
        enc->right[i] = wb_robot_get_device( dev_right_name );

        // Inicialize the motors
        wb_motor_set_position(enc->left[i], INFINITY);
        wb_motor_set_position(enc->right[i], INFINITY);
        wb_motor_set_velocity(enc->left[i], 0.0);
        wb_motor_set_velocity(enc->right[i], 0.0);
    }

    // Inicialize the other attributes of encoder
    enc->vel = 0.0;
    enc->rotvel = 0.0;
    enc->index = 0;
    enc->motors_count = motors_count;

    // Success
    link->enc_obj = enc;
    return UFR_OK;
}

static
void ufr_enc_motors_free(link_t* link) {
    if ( link->enc_obj ) {
        free(link->enc_obj);
    }
}

static
int ufr_enc_motors_clear(link_t* link) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    enc->vel = 0.0;
    enc->rotvel = 0.0;
    enc->index = 0;
    return UFR_OK;
}

static
int ufr_enc_motors_put_u32(link_t* link, const uint32_t val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = (double) val[wrote]; break;
            case 1: enc->rotvel = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_put_i32(link_t* link, const int32_t val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = (double) val[wrote]; break;
            case 1: enc->rotvel = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_put_f32(link_t* link, const float val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    if ( enc == NULL ) {
        return ufr_error(link, -1, "Encoder is NULL");
    }

    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = (double) val[wrote]; break;
            case 1: enc->rotvel = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_put_u64(link_t* link, const uint64_t val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = (double) val[wrote]; break;
            case 1: enc->rotvel = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_put_i64(link_t* link, const int64_t val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = (double) val[wrote]; break;
            case 1: enc->rotvel = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_put_f64(link_t* link, const double val[], int nitems) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->vel = val[wrote]; break;
            case 1: enc->rotvel = val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_put_str(link_t* link, const char* val_str) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    const double val = atof(val_str);
    switch (enc->index) {
        case 0: enc->vel = val; break;
        case 1: enc->rotvel = val; break;
        default: break;
    }
    enc->index += 1;
    return UFR_OK;
}


static
int ufr_enc_motors_cmd_send(link_t* link) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    const double speed_left = enc->vel - (enc->rotvel * 0.125); // HALF_DISTANCE_BETWEEN_WHEELS
    const double speed_right = enc->vel + (enc->rotvel * 0.125); // HALF_DISTANCE_BETWEEN_WHEELS

    // Send the speed to the WeBots
    for ( uint8_t i=0; i<enc->motors_count; i++ ) {
        wb_motor_set_velocity(enc->left[i], speed_left);
        wb_motor_set_velocity(enc->right[i], speed_right);
    }

    ufr_log(link, "linear: %f, rotação: %f", enc->vel, enc->rotvel);
    ufr_enc_motors_clear(link);
    return UFR_OK;
}


static
int ufr_enc_motors_put_cmd(link_t* link, char cmd) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    if ( cmd == '\n' ) {
        ufr_enc_motors_cmd_send(link);
        ufr_enc_motors_clear(link);
    }
    return UFR_OK;
}

static
int ufr_enc_motors_enter(link_t* link, size_t maxsize) {
    return -1;
}

static
int ufr_enc_motors_leave(link_t* link) {
    return -1;
}

static
int ufr_enc_motors_next(link_t* link) {
    enc_motors_t* enc = (enc_motors_t*) link->enc_obj;
    if ( enc->index >= 2 ) {
        return -1;
    }
    enc->index += 1;
    return UFR_OK;
}



static
int ufr_enc_motors_cmd_eof(link_t* link) {
    return UFR_OK;
}

static
ufr_enc_api_t ufr_enc_motors_api = {
    .init = ufr_enc_motors_init,
    .free = ufr_enc_motors_free,

    .put_u32 = ufr_enc_motors_put_u32,
    .put_i32 = ufr_enc_motors_put_i32,
    .put_f32 = ufr_enc_motors_put_f32,

    .put_u64 = ufr_enc_motors_put_u64,
    .put_i64 = ufr_enc_motors_put_i64,
    .put_f64 = ufr_enc_motors_put_f64,

    .put_cmd = ufr_enc_motors_put_cmd,
    .put_str = ufr_enc_motors_put_str,
    .put_raw = NULL,
    .put_bin = NULL,

    // Commands
    .cmd_enter = ufr_enc_motors_enter,
    .cmd_leave = ufr_enc_motors_leave,
    .cmd_next = ufr_enc_motors_next,
    .cmd_clear = ufr_enc_motors_clear,
    .cmd_send = ufr_enc_motors_cmd_send,
    .cmd_eof = ufr_enc_motors_cmd_eof
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_enc_webots_new_motors(link_t* link, int type, const ufr_args_t* args) {
    link->enc_api = &ufr_enc_motors_api;
    return UFR_OK;
}