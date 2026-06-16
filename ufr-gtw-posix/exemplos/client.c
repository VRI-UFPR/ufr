#include <ufr.h>
#include <stdio.h>
#include <unistd.h>

int main() {
    link_t client = ufr_client("@new socket @coder msgpack @port %d @log 10", 7000);

    for (int i=0; i<2; i++) {
        int resultado = 0;
        ufr_connect(&client);
        ufr_put(&client, "%d %s\n\n", 10, "time");
        ufr_get(&client, "> %d\n", &resultado);
        printf("Resultado %d\n", resultado);
        usleep(1000000);
    }

    ufr_close(&client);
}

