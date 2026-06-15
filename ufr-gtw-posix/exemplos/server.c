#include <ufr.h>
#include <stdio.h>


int main() {
    int time_now = 10;
    link_t server = ufr_server_st("@new socket @coder msgpack @port %d @log 3", 7000);
    while ( ufr_loop() ) {
        if ( ufr_accept(&server) ) {
            int code;
            char comando[1024];
            const int res = ufr_get(&server, "> %d %s\n", &code, comando);
            ufr_put(&server, "%d\n\n", time_now);
            time_now += 1;
        }
    }
    printf("FIM\n");
    ufr_close(&server);
}

