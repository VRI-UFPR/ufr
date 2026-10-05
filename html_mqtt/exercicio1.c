#include <stdio.h>
#include <ufr.h>

int main() {
    // Abre 
    UFILE* fd = ufr_publisher("@new mqtt @coder msgpack @host klaso.cc @topic teste");

    // 
    for (int i=0; i<10; i++) {
        float val = 2.0 * i;
        ufr_put(fd, "x=%f y=%f\n", val, 1.0);
        sleep(1);
    }

    // Fim
    return 0;
}

