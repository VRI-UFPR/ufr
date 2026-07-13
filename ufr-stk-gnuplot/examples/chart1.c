#include <ufr.h>
#include <stdio.h>
#include <math.h>

int ufr_gtw_gnuplot_new(link_t* link, int type);

int main() {
    ufr_stdout("@new %p", ufr_gtw_gnuplot_new);

    // Loop principal
    for ( int i=0; i<10; i++ ) {
        float t = i*0.1;
        float value = cos(t);
        ufr_printf("%f %f\n", t, 0.0);
        usleep(500000);
    }

    // fim
    return 0;
}


