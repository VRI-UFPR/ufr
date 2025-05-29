#include <stdio.h>
#include <string.h>
#include <ufr.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>

int ufr_gtw_chart_new(link_t* link, int type);

int main() {
    link_t chart = ufr_publisher("@new %p", ufr_gtw_chart_new);
    // link_t chart2 = ufr_publisher("@new %p", ufr_gtw_chart_new);

    int i=0;
    while ( ufr_loop_ok() ) {
        const float idx = i*0.125;
        const float rnd = (rand() % 100) / 1000.0;
        ufr_put(&chart, "fff\n", sin(idx) * idx, 2.0 + rnd, 1.0 );
        // ufr_put(&chart2, "fff\n", sin(idx) * idx, 2.0, rnd);
        usleep(200000);
        i += 1;
    }

    // ufr_close(&chart);
    return 0;
}

