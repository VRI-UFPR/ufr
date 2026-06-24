// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

// global variables
bool cap_is_started = false;
cv::VideoCapture cap;
link_t* video = NULL;

// constantes
const char* args_frame = "@new mqtt @host 177.153.62.174 @coder msgpack @topic /pioneer/camera/frame";
const char* args_ctrl  = "@new mqtt @host 177.153.62.174 @coder msgpack @topic /pioneer/camera/ctrl";

// ============================================================================
//  Functions
// ============================================================================

void camera_start() {
    printf("START\n");
    const bool res = cap.open(0);
    if ( res == false ) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);    
    cap_is_started = true;
    video = ufr_publisher(args_frame);
}

void camera_stop() {
    printf("STOP\n");
    if ( cap_is_started ) {
        cap.release();
        cap_is_started = false;
        ufr_close(video);
    }
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    link_t* control = ufr_subscriber(args_ctrl);

    Mat frame;
    int count = 0;
    std::vector<uint8_t> buffer;
    while( ufr_loop() ) {
        if ( cap_is_started ) {
            cap >> frame;

            if ( ufr_recv_async(control) ) {
                char buffer[1024];
                ufr_get(control, "%s", buffer);
                if ( strncmp(buffer, "start", 5) == 0 ) {
                    camera_start();
                } else if ( strncmp(buffer, "stop", 4) == 0 ) {
                    camera_stop();
                } else {
                    printf("Comando invalido (%s)\n", buffer);
                }
            }

            if (frame.empty()) {
                std::cerr << "Error: Blank frame grabbed." << std::endl;
                break;
            }

            count += 1;
            if ( count < 25 ) {
                continue;
            }

            count = 0;
            printf("%d %d\n", frame.cols, frame.rows);
            imencode(".jpg", frame, buffer);
            ufr_put_bin(video, "image/jpeg", (const char*) &buffer[0], buffer.size());
            ufr_send(video);

        } else {
            printf("Esperando START para iniciar a camera\n");
            char buffer[1024];
            ufr_get(control, "> %s", buffer);
            if ( strncmp(buffer, "start", 5) == 0 ) {
                camera_start();
            } else if ( strncmp(buffer, "stop", 4) == 0 ) {
                camera_stop();
            } else {
                printf("Comando invalido (%s)\n", buffer);
            }
        }

    }

    ufr_close(video);
    return 0;
}