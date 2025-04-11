// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

struct Gateway {
    VideoCapture capture;
    Mat frame;
};

// ============================================================================
//  Decoder OpenCV
// ============================================================================

static
int ufr_dcr_opencv_boot(link_t* link, const ufr_args_t* args) {
    link->dcr_obj_idx = 0;
    return UFR_OK;
}

static
void ufr_dcr_opencv_close(link_t* link) {

}

static
int ufr_dcr_opencv_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    link->dcr_obj_idx = 0;
    return UFR_OK;
}

static
int ufr_dcr_opencv_get_nbytes(link_t* link) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    return gtw->frame.total();
}

static
int ufr_dcr_opencv_get_nitems(link_t* link) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    return gtw->frame.total();
}

static
uint8_t* ufr_dcr_opencv_get_rawptr(link_t* link) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    return (uint8_t*) gtw->frame.data;
}

static
int ufr_dcr_opencv_get_u32(link_t* link, uint32_t* val, int maxlen) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    switch (link->dcr_obj_idx) {
        case 0: val[0] = gtw->frame.type();  break;
        case 1: val[0] = gtw->frame.rows;  break;
        case 2: val[0] = gtw->frame.cols;  break;
        default: break;
    }
    link->dcr_obj_idx += 1;
    return 1;
}

static
int ufr_dcr_opencv_get_i32(link_t* link, int32_t* val, int maxlen) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    switch (link->dcr_obj_idx) {
        case 0: val[0] = gtw->frame.type(); break;
        case 1: val[0] = gtw->frame.rows; break;
        case 2: val[0] = gtw->frame.cols; break;
        default: val[0] = 0; break;
    }
    link->dcr_obj_idx += 1;
    return 1;
}

static
int ufr_dcr_opencv_get_f32(link_t* link, float* ret_val, int maxlen) {
    return UFR_OK;
}

static
int ufr_dcr_opencv_get_str(link_t* link, char* val, int maxlen) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    if ( link->dcr_obj_idx == 0 ) {
        const int type = gtw->frame.type();
        if ( type == CV_8UC1 ) {
            strcpy(val, "CV_8UC1");
        } else if ( type == CV_8UC3 ) {
            strcpy(val, "CV_8UC3");
        }
        link->dcr_obj_idx += 1;
    }
    return UFR_OK;
}

static
int ufr_dcr_opencv_enter(link_t* link) {
    return UFR_OK;
}

static
int ufr_dcr_opencv_leave(link_t* link) {
    return UFR_OK;
}

static
ufr_dcr_api_t ufr_dcr_opencv_api = {
    .boot = ufr_dcr_opencv_boot,
    .close = ufr_dcr_opencv_close,
    .recv_cb = ufr_dcr_opencv_recv_cb,
    .recv_async_cb = ufr_dcr_opencv_recv_cb,
    .next = NULL,

    .get_type = NULL,
    .get_nbytes = ufr_dcr_opencv_get_nbytes,
    .get_nitems = ufr_dcr_opencv_get_nitems,
    .get_rawptr = ufr_dcr_opencv_get_rawptr,

    .get_raw = NULL,
    .get_str = ufr_dcr_opencv_get_str,

    .get_u32 = ufr_dcr_opencv_get_u32,
    .get_i32 = ufr_dcr_opencv_get_i32,
    .get_f32 = ufr_dcr_opencv_get_f32,

    .get_u64 = NULL,
    .get_i64 = NULL,
    .get_f64 = NULL,

    .enter = ufr_dcr_opencv_enter,
    .leave = ufr_dcr_opencv_leave
};

// ============================================================================
//  Gateway OpenCV
// ============================================================================

static
int    ufr_gtw_opencv_type(const link_t* link) {
    return UFR_OK;
}

static
int    ufr_gtw_opencv_state(const link_t* link) {
    return UFR_OK;
}

static
size_t ufr_gtw_opencv_size(const link_t* link, int type) {
    return 0;
}

static
int  ufr_gtw_opencv_boot(link_t* link, const ufr_args_t* args) {
    Gateway* gtw = new Gateway();
    link->gtw_obj = gtw;
    link->gtw_shr = NULL;
    return UFR_OK;
}

static
int  ufr_gtw_opencv_start(link_t* link, int type, const ufr_args_t* args) {
    Gateway* gtw = (Gateway*) link->gtw_obj;

    if ( type == UFR_START_SUBSCRIBER ) {
        link->dcr_api = &ufr_dcr_opencv_api;
        ufr_boot_dcr(link, args);

        char buffer[UFR_ARGS_TOKEN];
        const char* filename = ufr_args_gets(args, buffer, "@file", NULL);
        if ( filename != NULL ) {
            gtw->capture.open(filename);
            if ( !gtw->capture.isOpened() ) {
                return -1;
            }
        } else {
            const int idx = ufr_args_geti(args, "@id", -1);
            if ( idx >= 0 ) {
                gtw->capture.open(idx);
                if ( !gtw->capture.isOpened() ) {
                    return -1;
                }
            } else {
                return -1;
            }
        }
    }

    return UFR_OK;
}

static
void ufr_gtw_opencv_stop(link_t* link, int type) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    gtw->capture.release();
}

static
int  ufr_gtw_opencv_copy(link_t* link, link_t* out) {
    return UFR_OK;
}

static
size_t ufr_gtw_opencv_read(link_t* link, char* buffer, size_t length) {
    return 0;
}

static
size_t ufr_gtw_opencv_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

static
int ufr_gtw_opencv_recv(link_t* link) {
    Gateway* gtw = (Gateway*) link->gtw_obj;
    gtw->capture.read(gtw->frame);
    if ( gtw->frame.empty() ) {
        return -1;
    }
    link->dcr_api->recv_cb(link, NULL, 0);
    return UFR_OK;
}

static
int ufr_gtw_opencv_recv_async(link_t* link) {
    // link->dcr_api->recv_async_cb(link, NULL, 0);
    return -1;
}

static
int ufr_gtw_opencv_accept(link_t* link, link_t* out_client) {
    return UFR_OK;
}

static
const char* ufr_gtw_opencv_test_args(const link_t* link) {
    return "";
}

// ============================================================================
//  Public
// ============================================================================

extern "C" {

ufr_gtw_api_t ufr_gtw_opencv_api = {
    .name = "Camera:OpenCV",

    .type = ufr_gtw_opencv_type,
    .state = ufr_gtw_opencv_state,
    .size = ufr_gtw_opencv_size,

    .boot = ufr_gtw_opencv_boot,
    .start = ufr_gtw_opencv_start,
    .stop = ufr_gtw_opencv_stop,
    .copy = ufr_gtw_opencv_copy,

    .read = ufr_gtw_opencv_read,
    .write = ufr_gtw_opencv_write,

    .recv = ufr_gtw_opencv_recv,
    .recv_async = ufr_gtw_opencv_recv_async,
    .recv_peer_name = NULL,

    .accept = ufr_gtw_opencv_accept,

    .test_args = ufr_gtw_opencv_test_args,
};

}

