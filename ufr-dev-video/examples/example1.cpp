// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


// ============================================================================
//  Test
// ============================================================================

int main() {
    // link_t video = ufr_subscriber("@new video @@new zmq:topic @@coder msgpack @@port 3000");
    link_t video = ufr_subscriber("@new video @id 0");

    // link_t video = ufr_subscriber("@new video @@new mqtt @@coder msgpack @@host 10.0.0.6 @@topic camera1");
    // link_t video = ufr_subscriber("@new video @@new mqtt @@coder msgpack @@host 185.159.82.136 @@topic camera");
    // link_t video = ufr_subscriber("@new video @@new ros_humble @@coder ros_humble:image @@topic camera_rgb");
    // link_t video = ufr_subscriber("@new video @file teste1.mp4");

    while(1) {
        if ( ufr_recv(&video) != UFR_OK ) {
            break;
        }

        int cols = ufr_meta_i32(&video, 11);
        int rows = ufr_meta_i32(&video, 12);
        printf("%d %d\n", cols, rows);

        /*int type;
        int size[2];
        ufr_get(&video, "iii", &type, &size[0], &size[1]);
        void* data = (void*) ufr_get_rawptr(&video);
        Mat image(2, size, type, data, 0);
        imshow("janela", image);
        waitKey(1);*/
    }

    ufr_close(&video);
    return 0;
}