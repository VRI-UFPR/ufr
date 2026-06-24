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
    // Open publisher link
    link_t* video = ufr_publisher("@new mqtt @coder msgpack @topic camera1");

    // Open the camera
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    // Set size of the camera
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Main loop
    int count = 0;
    std::vector<uint8_t> buffer;
    while( ufr_loop() ) {
        // Read the frame from the camera
        Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Blank frame grabbed." << std::endl;
            break;
        }

        // Decrease the framerate
        count += 1;
        if ( count < 25 ) {
            continue;
        }

        // Send the image
        count = 0;
        printf("%d %d\n", frame.cols, frame.rows);
        imencode(".jpg", frame, buffer);
        ufr_put_file(video, "image/jpeg", (const char*) &buffer[0], buffer.size());
        ufr_send(video);

    }

    // End
    ufr_close(video);
    return 0;
}