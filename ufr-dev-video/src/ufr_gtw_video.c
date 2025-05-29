// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>

extern ufr_gtw_api_t ufr_gtw_link_api;
extern ufr_gtw_api_t ufr_gtw_opencv_api;


// ============================================================================
//  Public Function
// ============================================================================

int ufr_gtw_video_new(link_t* link, int type, const ufr_args_t* args) {
    // Case Subscriber
    if ( type == UFR_START_SUBSCRIBER ) {
        // 1. read from a video file
        char buffer[UFR_ARGS_TOKEN];
        const char* file = ufr_args_gets(args, buffer, "@file", NULL);
        if ( file != NULL ) {
            link->gtw_api = &ufr_gtw_opencv_api;

        // 2. read from a camera
        } else {
            const int id = ufr_args_geti(args, "@id", -1);
            if ( id >= 0 ) {
                link->gtw_api = &ufr_gtw_opencv_api;
                return UFR_OK;

            // 3. read from a other link
            } else {
                link->gtw_api = &ufr_gtw_link_api;
            }
        }
        return UFR_OK;
    }

    // Error
    return -1;
}