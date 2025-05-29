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
#include <webots/motor.h>
#include <webots/robot.h>
#include <string.h>
// #include <stdio.h>

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

// Constants, empirically found.
const double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
const double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
const double k_vertical_p = 3.0;        // P constant of the vertical PID.
const double k_roll_p = 50.0;           // P constant of the roll PID.
const double k_pitch_p = 30.0;          // P constant of the pitch PID.


typedef struct {
    WbDeviceTag front_left;
    WbDeviceTag front_right;
    WbDeviceTag rear_left;
    WbDeviceTag rear_right;
    double linear_x, linear_y, linear_z, angular_z;
    uint8_t index;
} enc_motors_drone_t;

// ============================================================================
//  Encoder
// ============================================================================

static
int ufr_enc_motors_drone_boot(link_t* link, const ufr_args_t* args) {
    char buffer1[UFR_ARGS_TOKEN];
    char buffer2[UFR_ARGS_TOKEN];
    char buffer3[UFR_ARGS_TOKEN];
    char buffer4[UFR_ARGS_TOKEN];
    const char* dev_tag1_name = ufr_args_gets(args, buffer1, "@tag1", "front left propeller");
    const char* dev_tag2_name = ufr_args_gets(args, buffer2, "@tag2", "front right propeller");
    const char* dev_tag3_name = ufr_args_gets(args, buffer3, "@tag3", "front left propeller");
    const char* dev_tag4_name = ufr_args_gets(args, buffer4, "@tag4", "front right propeller");


    WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
    WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
    WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
    WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");

    // Prepare encoder
    ufr_log_ini(link, "Inicializando encoder para motores (%s, %s)", dev_tag1_name, dev_tag2_name);
    enc_motors_drone_t* enc = malloc(sizeof(enc_motors_drone_t));
    enc->front_left = wb_robot_get_device( dev_tag1_name );
    enc->front_right = wb_robot_get_device( dev_tag2_name );
    enc->rear_left = wb_robot_get_device( dev_tag3_name );
    enc->rear_right = wb_robot_get_device( dev_tag4_name );


    enc->linear_x = 0.0;
    enc->linear_y = 0.0;
    enc->linear_z = 0.0;
    enc->angular_z = 0.0;
    enc->index = 0;    

    // Start the WeBots encoders
    wb_motor_set_position(enc->front_left, INFINITY);
    wb_motor_set_position(enc->front_right, INFINITY);
    wb_motor_set_position(enc->rear_left, INFINITY);
    wb_motor_set_position(enc->rear_right, INFINITY);

    wb_motor_set_velocity(enc->front_left, 0.0);
    wb_motor_set_velocity(enc->front_right, 0.0);
    wb_motor_set_velocity(enc->rear_left, 0.0);
    wb_motor_set_velocity(enc->rear_right, 0.0);

    // Success
    link->enc_obj = enc;
    ufr_log_end(link, "Inicializado encoder para motores (%s, %s, %s, %s)", 
        dev_tag1_name, dev_tag2_name, dev_tag3_name, dev_tag4_name);
    return UFR_OK;
}

static
void ufr_enc_motors_drone_close(link_t* link) {
    if ( link->enc_obj ) {
        free(link->enc_obj);
        link->enc_obj = NULL;
    }
}

static
void ufr_enc_motors_drone_clear(link_t* link) {
    enc_motors_drone_t* enc = (enc_motors_drone_t*) link->enc_obj;
    enc->linear_x = 0.0;
    enc->linear_y = 0.0;
    enc->linear_z = 0.0;
    enc->angular_z = 0.0;
    enc->index = 0;
}

static
int ufr_enc_motors_drone_put_u32(link_t* link, const uint32_t val[], int nitems) {
    enc_motors_drone_t* enc = (enc_motors_drone_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->linear_x = (double) val[wrote]; break;
            case 1: enc->linear_y = (double) val[wrote]; break;
            case 2: enc->linear_z = (double) val[wrote]; break;
            case 3: enc->angular_z = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_drone_put_i32(link_t* link, const int32_t val[], int nitems) {
    enc_motors_drone_t* enc = (enc_motors_drone_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->linear_x = (double) val[wrote]; break;
            case 1: enc->linear_y = (double) val[wrote]; break;
            case 2: enc->linear_z = (double) val[wrote]; break;
            case 3: enc->angular_z = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_drone_put_f32(link_t* link, const float val[], int nitems) {
    enc_motors_drone_t* enc = (enc_motors_drone_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->linear_x = (double) val[wrote]; break;
            case 1: enc->linear_y = (double) val[wrote]; break;
            case 2: enc->linear_z = (double) val[wrote]; break;
            case 3: enc->angular_z = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_drone_put_u64(link_t* link, const uint64_t val[], int nitems) {
    enc_motors_drone_t* enc = (enc_motors_drone_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->linear_x = (double) val[wrote]; break;
            case 1: enc->linear_y = (double) val[wrote]; break;
            case 2: enc->linear_z = (double) val[wrote]; break;
            case 3: enc->angular_z = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return UFR_OK;
}

static
int ufr_enc_motors_drone_put_i64(link_t* link, const int64_t val[], int nitems) {
    enc_motors_drone_t* enc = (enc_motors_drone_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->linear_x = (double) val[wrote]; break;
            case 1: enc->linear_y = (double) val[wrote]; break;
            case 2: enc->linear_z = (double) val[wrote]; break;
            case 3: enc->angular_z = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return UFR_OK;
}

static
int ufr_enc_motors_drone_put_f64(link_t* link, const double val[], int nitems) {
    enc_motors_drone_t* enc = (enc_motors_drone_t*) link->enc_obj;
    int wrote = 0;
    for (; wrote < nitems; wrote++) {
        switch (enc->index) {
            case 0: enc->linear_x = (double) val[wrote]; break;
            case 1: enc->linear_y = (double) val[wrote]; break;
            case 2: enc->linear_z = (double) val[wrote]; break;
            case 3: enc->angular_z = (double) val[wrote]; break;
            default: break;
        }
        enc->index += 1;
    }
    return wrote;
}

static
int ufr_enc_motors_drone_put_str(link_t* link, const char* val_str) {
    enc_motors_drone_t* enc = (enc_motors_drone_t*) link->enc_obj;
    const double val = atof(val_str);
    switch (enc->index) {
        case 0: enc->linear_x = (double) val; break;
        case 1: enc->linear_y = (double) val; break;
        case 2: enc->linear_z = (double) val; break;
        case 3: enc->angular_z = (double) val; break;
        default: break;
    }
    enc->index += 1;
    return UFR_OK;
}

static
int ufr_enc_motors_drone_put_cmd(link_t* link, char cmd) {
    enc_motors_drone_t* enc = (enc_motors_drone_t*) link->enc_obj;
    if ( cmd == '\n' ) {
        const double roll_disturbance = 0;
        const double pitch_disturbance = 0;
        const double yaw_disturbance = 0;
        const double target_altitude = 1.0;

        const double roll = 0; //wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
        const double pitch = 0; //wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
        const double altitude = 0; //wb_gps_get_values(gps)[2];
        const double roll_velocity = 0; //wb_gyro_get_values(gyro)[0];
        const double pitch_velocity = 0; //wb_gyro_get_values(gyro)[1];

        // Compute the roll, pitch, yaw and vertical inputs.
        const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
        const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
        const double yaw_input = yaw_disturbance;
        const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
        const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

        // Actuate the motors taking into consideration all the computed inputs.
        const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
        const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
        const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
        const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;

        // const double speed_left = enc->vel - enc->rotvel * 0.125; // HALF_DISTANCE_BETWEEN_WHEELS
        // const double speed_right = enc->vel + enc->rotvel * 0.125; // HALF_DISTANCE_BETWEEN_WHEELS
        wb_motor_set_velocity(enc->front_left, 0);
        wb_motor_set_velocity(enc->front_right, 0);
        wb_motor_set_velocity(enc->rear_left, 0);
        wb_motor_set_velocity(enc->rear_right, 0);

        // ufr_log(link, "Motor vel: %f, rotvel: %f", enc->vel, enc->rotvel);
        ufr_enc_motors_drone_clear(link);
    }
    return UFR_OK;
}

static
int ufr_enc_motors_drone_enter(link_t* link, size_t maxsize) {
    return -1;
}

static
int ufr_enc_motors_drone_leave(link_t* link) {
    return -1;
}

static
ufr_enc_api_t ufr_enc_motors_drone_api = {
    .boot = ufr_enc_motors_drone_boot,
    .close = ufr_enc_motors_drone_close,
    .clear = ufr_enc_motors_drone_clear,

    .put_u32 = ufr_enc_motors_drone_put_u32,
    .put_i32 = ufr_enc_motors_drone_put_i32,
    .put_f32 = ufr_enc_motors_drone_put_f32,

    .put_u64 = ufr_enc_motors_drone_put_u64,
    .put_i64 = ufr_enc_motors_drone_put_i64,
    .put_f64 = ufr_enc_motors_drone_put_f64,

    .put_str = ufr_enc_motors_drone_put_str,
    .put_cmd = ufr_enc_motors_drone_put_cmd,

    .enter = ufr_enc_motors_drone_enter,
    .leave = ufr_enc_motors_drone_leave,
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_enc_webots_new_motors_drone(link_t* link, int type) {
    link->enc_api = &ufr_enc_motors_drone_api;
    return UFR_OK;
}