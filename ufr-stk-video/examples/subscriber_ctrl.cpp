// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include <unistd.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

const char* args_frame = "@new video @cols 640 @rows 480 @@new mqtt @@host 177.153.62.174 @@coder msgpack @@topic /pioneer/camera/frame";
const char* args_ctrl  = "@new mqtt @host 177.153.62.174 @coder msgpack @topic /pioneer/camera/ctrl";

// ============================================================================
//  Test
// ============================================================================

int main() {
    // link_t video = ufr_subscriber("@new video @id 0 @type gray");
    link_t video = ufr_subscriber("");


    link_t control = ufr_publisher("@new mqtt @coder msgpack @topic /pioneer/camera/control");
    ufr_put(&control, "%s\n", "start");

    while( ufr_loop() ) {
        if ( ufr_recv(&video) == false ) {
            break;
        }

        int type;
        int size[2];
        void* data;
        ufr_get(&video, "%d %d %d %p", &type, &size[0], &size[1], &data);
        printf("%d %d %p\n", size[0], size[1], data);

        // Show the image
        Mat image(2, size, type, data, 0);
        imshow("janela", image);
        waitKey(1);
    }

    printf("fim\n");
    ufr_put(&control, "%s\n", "stop");
    sleep(1);
    ufr_close(&control);
    ufr_close(&video);
    return 0;
}

