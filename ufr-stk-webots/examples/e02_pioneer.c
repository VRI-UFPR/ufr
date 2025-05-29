#include <stdio.h>
#include <ufr.h>
#include <unistd.h>

int main() {
    link_t motor = ufr_publisher("@new webots @topic /cmd_vel");
    link_t pose = ufr_subscriber("@new webots @topic /pose");
    link_t scan = ufr_subscriber("@new webots @topic /scan");

    float range[1024];
    while( ufr_loop_ok() ) {
        float x,y,th;
        // ufr_put(&motor, "ff\n", -0.5, 0);
        ufr_get(&pose, "^fff", &x, &y, &th);
        ufr_get(&scan, "^-------");
        /*const int range_nitems = ufr_get_af32(&scan, range, 4096);
        for (int i=0; i<range_nitems; i++) {
            printf("%f ", range[i]);
        }
        printf("\n");*/
    }

    ufr_close(&scan);
    ufr_close(&pose);
    ufr_close(&motor);
    return 0;
}