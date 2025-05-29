#include <stdio.h>
#include <ufr.h>
#include <unistd.h>

int main() {
    link_t motor = ufr_publisher("@new webots @topic /cmd_vel @log 4");
    link_t pose = ufr_subscriber("@new webots @topic /pose @log 4");


    ufr_header(&motor, "@vel f32 @rotvel f32")

    while( ufr_loop_ok() ) {
        float x,y,th;
        ufr_put(&motor, "ff\n", -0.5, 0);
        ufr_get(&pose, "^fff", &x, &y, &th);
        printf("%f %f %f\n", x,y,th);
    }

    ufr_close(&motor);
    ufr_close(&pose);
    return 0;
}