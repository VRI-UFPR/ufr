// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>

extern ufr_gtw_api_t ufr_gtw_link_api;
extern ufr_gtw_api_t ufr_gtw_opencv_api;

// ============================================================================
//  Codigo
// ============================================================================

int  ufr_gtw_video_boot(link_t* link, const ufr_args_t* args) {

    if ( ufr_link_is_subscriber(link) ) {
        // 1. read from a video file
        char buffer[UFR_ARGS_TOKEN];
        const char* file = ufr_args_gets(args, buffer, "@file", NULL);
        if ( file != NULL ) {
            link->gtw_api = &ufr_gtw_opencv_api;
            return link->gtw_api->boot(link, args);
        }

        // 2. read from a camera
        const int id = ufr_args_geti(args, "@id", -1);
        if ( id >= 0 ) {
            link->gtw_api = &ufr_gtw_opencv_api;
            return link->gtw_api->boot(link, args);
        }

        // 3. read from a other link
        link->gtw_api = &ufr_gtw_link_api;
        return link->gtw_api->boot(link, args);
    }

    return -1;
}

ufr_gtw_api_t ufr_gtw_video_api = {
    .type = NULL,
    .state = NULL,
    .size = NULL,

    .boot = ufr_gtw_video_boot,
    .start = NULL,
    .stop = NULL,
    .copy = NULL,

    .read = NULL,
    .write = NULL,

    .recv = NULL,
    .recv_async = NULL,

    .accept = NULL,

    .test_args = NULL,
};


// ============================================================================
//  Public Function
// ============================================================================

int ufr_gtw_video_new(link_t* link, int type) {
    ufr_link_init(link, &ufr_gtw_video_api);
    return UFR_OK;
}