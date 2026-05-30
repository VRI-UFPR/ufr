// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include <unistd.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

const char* args_frame = "@new video @cols 640 @rows 480 @@new mqtt @@coder msgpack @@topic camera @@host 177.153.62.174";

// ============================================================================
//  Test
// ============================================================================

int main() {
    link_t frame = ufr_subscriber(args_frame);

    while( ufr_loop() ) {
        if ( ufr_recv(&frame) == false ) {
            break;
        }

        int type;
        int size[2];
        void* data;
        ufr_get(&frame, "%d %d %d %p", &type, &size[0], &size[1], &data);
        printf("%d %d %p\n", size[0], size[1], data);

        // Show the image
        Mat image(2, size, type, data, 0);
        imshow("janela", image);
        waitKey(1);
    }

    // Fim
    printf("Fim\n");
    ufr_close(&frame);
    return 0;
}