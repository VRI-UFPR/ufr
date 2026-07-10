#include <ufr.h>
#include <stdio.h>
#include <math.h>

int ufr_gtw_gnuplot_new(link_t* link, int type);

int main() {
    ufr_stdout("@new mqtt @coder msgpack");

    // Loop principal
    double teste[10];
    for ( int i=0; i<1; i++ ) {
        float t = i*0.1;
        float value = cos(t);
        ufr_printf("%s %d %a:lf:?\n", "opaaa", 30, teste, 10);
        usleep(100000);
        printf("%d\n", i);
    }

    // fim
    return 0;
}


