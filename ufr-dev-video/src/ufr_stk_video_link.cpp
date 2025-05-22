// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include "opencv2/opencv.hpp"

#define PROTOCOL_BASIC  0
#define PROTOCOL_ROS    1

using namespace std;
using namespace cv;


struct GatewayLink {
    link_t link;
    std::vector<uint8_t> buffer;
    Mat frame;
    uint8_t protocol;
};

// ============================================================================
//  Encoder Link
// ============================================================================

static
int ufr_enc_link_boot(link_t* link, const ufr_args_t* args) {
	return UFR_OK;
}

static
void ufr_enc_link_close(link_t* link) {
    if ( link->dcr_obj != NULL ) {
        
    }
}

static
int ufr_enc_link_put_u32(link_t* link, const uint32_t val[], int nitems) {
    return UFR_OK;
}

static
int ufr_enc_link_put_i32(link_t* link, const int32_t val[], int nitems) {
    return UFR_OK;
}

static
int ufr_enc_link_put_f32(link_t* link, const float val[], int nitems) {
    return UFR_OK;
}

static
int ufr_enc_link_put_str(link_t* link, const char* val) {
    return UFR_OK;
}

static
int ufr_enc_link_put_cmd(link_t* link, char cmd) {
    return UFR_OK;
}

static
int ufr_enc_link_put_raw(link_t* link, const uint8_t* buffer, int size) {
    return UFR_OK;
}

int ufr_enc_link_enter(link_t* link, size_t maxsize) {
    return UFR_OK;
}


int ufr_enc_link_leave(link_t* link) {
    return UFR_OK;
}

ufr_enc_api_t ufr_enc_link_api = {
    .boot = ufr_enc_link_boot,
    .close = ufr_enc_link_close,
    .clear = NULL,

    .put_u32 = ufr_enc_link_put_u32,
    .put_i32 = ufr_enc_link_put_i32,
    .put_f32 = ufr_enc_link_put_f32,

    .put_u64 = NULL,
    .put_i64 = NULL,
    .put_f64 = NULL,

    .put_cmd = ufr_enc_link_put_cmd,
    .put_str = ufr_enc_link_put_str,
    .put_raw = ufr_enc_link_put_raw,

    .enter = ufr_enc_link_enter,
    .leave = ufr_enc_link_leave
};


// ============================================================================
//  Decoder Link
// ============================================================================

static
int ufr_dcr_link_boot(link_t* link, const ufr_args_t* args) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    return ufr_boot_dcr(&gtw->link, args);
}

static
void ufr_dcr_link_close(link_t* link) {
    
}

static
int ufr_dcr_link_recv_cb(link_t* link, char* msg_data, size_t msg_size) {
    link->dcr_obj_idx = 0;
    return 0;
}

int ufr_dcr_link_get_nbytes(link_t* link) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    return gtw->frame.total();
}

int ufr_dcr_link_get_nitems(link_t* link) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    return gtw->frame.cols;
}

uint8_t* ufr_dcr_link_get_raw_ptr(link_t* link) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    return (uint8_t*) gtw->frame.data;
}

static
int ufr_dcr_link_get_u32(link_t* link, uint32_t* val, int max_items) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
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
int ufr_dcr_link_get_i32(link_t* link, int32_t* val, int max_items) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
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
int ufr_dcr_link_get_f32(link_t* link, float* val, int max_items) {
    return UFR_OK;
}

static
int ufr_dcr_link_get_str(link_t* link, char* val, int max_bytes) {
    return UFR_OK;
}

static
int ufr_dcr_link_enter(link_t* link) {
    return UFR_OK;
}

static
int ufr_dcr_link_leave(link_t* link) {
    return UFR_OK;
}

ufr_dcr_api_t ufr_dcr_link_api = {
    .boot = ufr_dcr_link_boot,
    .close = ufr_dcr_link_close,
    .recv_cb = ufr_dcr_link_recv_cb,
    .recv_async_cb = ufr_dcr_link_recv_cb,
    .next = NULL,

    .get_type = NULL,
    .get_nbytes = ufr_dcr_link_get_nbytes,
    .get_nitems = ufr_dcr_link_get_nitems,
    .get_rawptr = ufr_dcr_link_get_raw_ptr,

    .get_raw = NULL,
    .get_str = ufr_dcr_link_get_str,

    .get_u32 = ufr_dcr_link_get_u32,
    .get_i32 = ufr_dcr_link_get_i32,
    .get_f32 = ufr_dcr_link_get_f32,

    .get_u64 = NULL,
    .get_i64 = NULL,
    .get_f64 = NULL,

    .enter = ufr_dcr_link_enter,
    .leave = ufr_dcr_link_leave
};


// ============================================================================
//  Gateway Link
// ============================================================================

int ufr_gtw_link_type(const link_t* link) {
    return UFR_OK;
}

int ufr_gtw_link_state(const link_t* link) {
    return UFR_OK;
}

size_t ufr_gtw_link_size(const link_t* link, int type) {
    return 0;
}

int  ufr_gtw_link_boot(link_t* link, const ufr_args_t* args) {
    GatewayLink* gtw = new GatewayLink();
    char link_text[1024];
    ufr_args_decrease_level(args->text, link_text);
    // printf("%s\n", link_text);

    char buffer[1024];
    ufr_args_t sub_args = {.text=link_text};
    std::string new_value = ufr_args_gets(&sub_args, buffer, "@new", "");
    if ( new_value == "" ) {
        gtw->protocol = PROTOCOL_BASIC;
    } else if ( new_value == "ros_humble" ) {
        gtw->protocol = PROTOCOL_ROS;
    } else {
        gtw->protocol = PROTOCOL_BASIC;
    }

    ufr_subscriber_args(&gtw->link, &sub_args);

    // gtw->data = malloc(1024*1024);
    link->gtw_obj = (void*) gtw;
    link->gtw_shr = NULL;

    link->dcr_api = &ufr_dcr_link_api;

    return UFR_OK;
}

int  ufr_gtw_link_start(link_t* link, int type, const ufr_args_t* args) {
    if ( type == UFR_START_SUBSCRIBER ) {
        GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
        return gtw->link.gtw_api->start(&gtw->link, type, args);
    }

    if ( type == UFR_START_PUBLISHER ) {
        return UFR_OK;
    }

    return 1;
}

void ufr_gtw_link_stop(link_t* link, int type) {
    // GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    // ufr_stop(&gtw->link);
}

int  ufr_gtw_link_copy(link_t* link, link_t* out) {
    return UFR_OK;
}

size_t ufr_gtw_link_read(link_t* link, char* buffer, size_t length) {
    return 0;
}

size_t ufr_gtw_link_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

int ufr_gtw_link_recv(link_t* link) {
    // recebe os dados
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    const int res = ufr_recv(&gtw->link);
    if ( res != UFR_OK ) {
        return res;
    }

    // limpa o index
    link->dcr_obj_idx = 0;

    // protocolo A
    if ( gtw->protocol == PROTOCOL_BASIC ) {
        char format[512];
        int rows, cols;
        ufr_get(&gtw->link, "sii", format, &rows, &cols);

        const int nbytes = ufr_get_nbytes(&gtw->link);
        if ( nbytes == 0 ){
            return ufr_error(link, -1, "Image has 0 bytes");
        }

        const uint8_t* rawptr = ufr_get_rawptr(&gtw->link);
        if ( rawptr == 0 ){
            return ufr_error(link, -1, "Image has NULL pointer for image");
        }

        std::vector<uint8_t> jpg_raw(rawptr, rawptr + nbytes);
        gtw->frame = imdecode(jpg_raw, cv::IMREAD_UNCHANGED);

    // protocolo para mensagem do ROS
    } else if ( gtw->protocol == PROTOCOL_ROS ) {
        char format[16];
        ufr_get(&gtw->link, "s", format);

        // printf("aaa %s\n", format);
        // mono8
        if ( strcmp(format, "mono8") == 0 ) {
            int size[2] = {480, 640};
            void* data = (void*) ufr_get_rawptr(&gtw->link);
            if ( data == NULL ) {
                return ufr_error(link, -1, "Image has NULL pointer for image");
            }
            gtw->frame = Mat(2, size, CV_8UC1, data, 0);

        // rgb8
        } else if ( strcmp(format, "rgb8") == 0 ) {
            int size[2] = {480, 640};
            void* data = (void*) ufr_get_rawptr(&gtw->link);
            if ( data == NULL ) {
                return ufr_error(link, -1, "Image has NULL pointer for image");
            }

            gtw->frame = Mat(2, size, CV_8UC3, data, 0);
            cvtColor(gtw->frame, gtw->frame, COLOR_RGB2BGR);
        }
    }

    // Success
    return UFR_OK;
}

int ufr_gtw_link_recv_async(link_t* link) {
    GatewayLink* gtw = (GatewayLink*) link->gtw_obj;
    const int res = ufr_recv_async(&gtw->link);
    if ( res != UFR_OK ) {
        return res;
    }
    // const size_t size = ufr_get_nbytes(&gtw->link);
    // gtw->buffer.resize(size);
    
    //
    link->dcr_obj_idx = 0;

    //
    char format[512];
    int rows, cols;
    ufr_get(&gtw->link, "sii", format, &rows, &cols);

// printf("%s %d %d\n", format, rows, cols);
    const int nbytes = ufr_get_nbytes(&gtw->link);
// printf("%d\n", nbytes);

    if ( nbytes == 0 ){
        return ufr_error(link, -1, "Image has 0 bytes");
    }

    const uint8_t* rawptr = ufr_get_rawptr(&gtw->link);
    if ( rawptr == 0 ){
        return ufr_error(link, -1, "Image has NULL pointer for image");
    }

    std::vector<uint8_t> jpg_raw(rawptr, rawptr + nbytes);
    gtw->frame = imdecode(jpg_raw, cv::IMREAD_UNCHANGED);

    return UFR_OK;
}

int ufr_gtw_lin_send(link_t* link) {
    return UFR_OK;
}

int ufr_gtw_link_accept(link_t* link, link_t* out_client) {
    return UFR_OK;
}

// tests
const char* ufr_gtw_link_test_args(const link_t* link) {
    return "";
}

// ============================================================================
//  Public
// ============================================================================

extern "C" {

ufr_gtw_api_t ufr_gtw_link_api = {
    .name = "Camera:Link",

    .type = ufr_gtw_link_type,
    .state = ufr_gtw_link_state,
    .size = ufr_gtw_link_size,

    .boot = ufr_gtw_link_boot,
    .start = ufr_gtw_link_start,
    .stop = ufr_gtw_link_stop,
    .copy = ufr_gtw_link_copy,

    .read = ufr_gtw_link_read,
    .write = ufr_gtw_link_write,

    .recv = ufr_gtw_link_recv,
    .recv_async = ufr_gtw_link_recv_async,
    .recv_peer_name = NULL,

    .accept = ufr_gtw_link_accept,

    .test_args = ufr_gtw_link_test_args,
};

}


