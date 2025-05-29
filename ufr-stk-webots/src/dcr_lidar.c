// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/device.h>
#include <ufr.h>

#include "ufr_webots.h"

typedef struct {
    WbDeviceTag lidar;
    float const* range_ptr;
    uint16_t range_size;

    float angle_min, angle_max, angle_increment;
    float time_increment, scan_time;

    float range_min, range_max;
    uint16_t index;
    uint16_t index2;
} decoder_t;

/*
float angle_min
float angle_max
float angle_increment
float time_increment
float scan_time
float range_min
float range_max
float[] ranges
float[] intensities
*/

// ============================================================================
//  Decoder
// ============================================================================

static
int ufr_dcr_lidar_boot(link_t* link, const ufr_args_t* args) {
    // new decoder
    decoder_t* dcr = malloc(sizeof(decoder_t));
    dcr->index = 0;
    dcr->index2 = 0;
    dcr->range_ptr = NULL;

    // get sensor for both wheel
    char buffer[UFR_ARGS_TOKEN];
    const char* dev_tag_name = ufr_args_gets(args, buffer, "@tag", NULL);

    // Search the lidar, case tag is not offer
    if ( dev_tag_name == NULL ) {
        const int n_devices = wb_robot_get_number_of_devices();
        for(int i=0; i<n_devices; i++) {
            WbDeviceTag tag = wb_robot_get_device_by_index(i);
            WbNodeType type = wb_device_get_node_type(tag);

            if ( type == WB_NODE_LIDAR ) {
                dev_tag_name = wb_device_get_name(tag);
                break;
            }
        }
    }

    if ( dev_tag_name == NULL ) {
        return ufr_error(link, -1, "Lidar not found");
    }

    // Open lidar
    const int time_step = ufr_gtw_webots_get_time_step();
    dcr->lidar = wb_robot_get_device(dev_tag_name);
    dcr->range_size = 0;
    wb_lidar_enable(dcr->lidar, time_step);

    // success
    ufr_log(link, "Opened the device %s", dev_tag_name);
    link->dcr_obj = dcr;
    return UFR_OK;
}

static
void ufr_dcr_lidar_close(link_t* link) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr ) {
        free(dcr);
        link->dcr_obj = NULL;
    }
}

static
char ufr_dcr_lidar_get_type(link_t* link) {
    return 'f';
}

static
int ufr_dcr_lidar_get_nitems(link_t* link) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->range_ptr ) {
        return dcr->range_size;
    }
    return 0;
}

static
int ufr_dcr_lidar_get_nbytes(link_t* link) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->range_ptr ) {
        return dcr->range_size * sizeof(float);
    }
    return 0;
}

static
uint8_t* ufr_dcr_lidar_get_raw_ptr(link_t* link) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    return (uint8_t*) dcr->range_ptr;
}

static
int ufr_dcr_lidar_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr ) {
        dcr->range_ptr = wb_lidar_get_range_image(dcr->lidar);
        dcr->range_size = wb_lidar_get_horizontal_resolution(dcr->lidar);
        dcr->range_max = wb_lidar_get_max_range(dcr->lidar);
        dcr->index = 0;
    }
    return UFR_OK;
}

static
int ufr_dcr_lidar_next(link_t* link) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->index > 8 ) {
        return -1;
    }
    dcr->index += 1;
    return UFR_OK;
}

static
int ufr_dcr_lidar_get_u32(link_t* link, uint32_t val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->range_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        if ( dcr->index < dcr->range_size ) {
            val[wrote] = dcr->range_ptr[dcr->index];
            dcr->index += 1;
        }
    }
    return wrote;
}

static
int ufr_dcr_lidar_get_i32(link_t* link, int32_t val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->range_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        if ( dcr->index < dcr->range_size ) {
            val[wrote] = dcr->range_ptr[dcr->index];
            dcr->index += 1;
        }
    }
    return wrote;
}

static
int ufr_dcr_lidar_get_f32(link_t* link, float val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->range_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;

    //
    if ( dcr->index == 7 ) {
        for (; wrote<nitems; wrote++) {
            if ( dcr->index2 >= dcr->range_size ) {
                break;
            }
            val[wrote] = dcr->range_ptr[dcr->index2];
            dcr->index2 += 1;
        }
        return wrote;
    }

    //
    if ( dcr->index == 8 ) {
        for (; wrote<nitems; wrote++) {
            if ( dcr->index2 >= dcr->range_size ) {
                break;
            }
            val[wrote] = 0;
            dcr->index2 += 1;
        }
        return wrote;
    }

    //
    for (; wrote<nitems; wrote++) {
        if ( dcr->index > 6 ) {
            break;
        }
        switch(dcr->index) {
            case 0: val[wrote] = dcr->angle_min; break;
            case 1: val[wrote] = dcr->angle_max; break;
            case 2: val[wrote] = dcr->angle_increment; break;
            case 3: val[wrote] = dcr->time_increment; break;
            case 4: val[wrote] = dcr->scan_time; break;
            case 5: val[wrote] = dcr->range_min; break;
            case 6: val[wrote] = dcr->range_max; break;
            default: val[wrote] = 0;
        }
        dcr->index += 1;
    }
    return wrote;
}

static
int ufr_dcr_lidar_get_u64(link_t* link, uint64_t val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->range_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        if ( dcr->index < dcr->range_size ) {
            val[wrote] = dcr->range_ptr[dcr->index];
            dcr->index += 1;
        }
    }
    return wrote;
}

static
int ufr_dcr_lidar_get_i64(link_t* link, int64_t val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->range_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;
    for (; wrote<nitems; wrote++) {
        if ( dcr->index < dcr->range_size ) {
            val[wrote] = dcr->range_ptr[dcr->index];
            dcr->index += 1;
        }
    }
    return wrote;
}

static
int ufr_dcr_lidar_get_f64(link_t* link, double val[], int nitems) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    if ( dcr->range_ptr == NULL ) {
        return -1;
    }
    int wrote = 0;

    //
    if ( dcr->index == 7 ) {
        for (; wrote<nitems; wrote++) {
            if ( dcr->index2 >= dcr->range_size ) {
                break;
            }
            val[wrote] = dcr->range_ptr[dcr->index2];
            dcr->index2 += 1;
        }
        return wrote;
    }

    //
    if ( dcr->index == 8 ) {
        for (; wrote<nitems; wrote++) {
            if ( dcr->index2 >= dcr->range_size ) {
                break;
            }
            val[wrote] = 0;
            dcr->index2 += 1;
        }
        return wrote;
    }

    //
    for (; wrote<nitems; wrote++) {
        switch(dcr->index) {
            case 0: val[wrote] = dcr->angle_min; break;
            case 1: val[wrote] = dcr->angle_max; break;
            case 2: val[wrote] = dcr->angle_increment; break;
            case 3: val[wrote] = dcr->time_increment; break;
            case 4: val[wrote] = dcr->scan_time; break;
            case 5: val[wrote] = dcr->range_min; break;
            case 6: val[wrote] = dcr->range_max; break;
            default: val[wrote] = 0;
        }
        dcr->index += 1;
    }
    return wrote;
}


static
int ufr_dcr_lidar_get_str(link_t* link, char* ret_val, int size) {
    return UFR_OK;
}

static
int ufr_dcr_lidar_enter(link_t* link) {
    decoder_t* dcr = (decoder_t*) link->dcr_obj;
    dcr->index2 = 0;
    if ( dcr->index == 7 ) {
        return UFR_OK;
    }
    if ( dcr->index == 8 ) {
        return UFR_OK;
    }
    return -1;
}

static
int ufr_dcr_lidar_leave(link_t* link) {
    return UFR_OK;
}

static
int ufr_dcr_lidar_get_meta(link_t* link, int index, char type, item_t* out) {
    return UFR_OK;
}

static
ufr_dcr_api_t dcr_lidar_api = {
    .boot = ufr_dcr_lidar_boot,
    .close = ufr_dcr_lidar_close,
	.recv_cb = ufr_dcr_lidar_recv_cb,
    .recv_async_cb = ufr_dcr_lidar_recv_cb,

    .next = ufr_dcr_lidar_next,

    .get_type = ufr_dcr_lidar_get_type,
    .get_nbytes = ufr_dcr_lidar_get_nbytes,
    .get_nitems = ufr_dcr_lidar_get_nitems,
    .get_rawptr = ufr_dcr_lidar_get_raw_ptr,

	.get_u32 = ufr_dcr_lidar_get_u32,
	.get_i32 = ufr_dcr_lidar_get_i32,
	.get_f32 = ufr_dcr_lidar_get_f32,

	.get_u64 = ufr_dcr_lidar_get_u64,
	.get_i64 = ufr_dcr_lidar_get_i64,
	.get_f64 = ufr_dcr_lidar_get_f64,

	.get_str = ufr_dcr_lidar_get_str,

    .enter = ufr_dcr_lidar_enter,
    .leave = ufr_dcr_lidar_leave
};

// ============================================================================
//  Public Function
// ============================================================================

int ufr_dcr_webots_new_lidar(link_t* link, int type) {
    link->dcr_api = &dcr_lidar_api;
    return UFR_OK;
}