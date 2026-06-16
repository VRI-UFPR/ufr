#include <ufr.h>
#include <stdio.h>
#include <stdlib.h>



link_t processos[5];


bool is_process_ok(int id) {
    printf("Testando o processo %d\n", id);
    int resultado;
    link_t* sendto = &processos[id];
    if ( ufr_connect(sendto) ) {
        ufr_put(sendto, "%d\n\n", 1);
        ufr_get(sendto, "> %d\n", &resultado);
        if ( resultado == 99 ) {
            printf("OK %d\n", id);
            return true;
        } else {
            printf("ERRO %d\n", id);
            return false;
        }
    } else {
        printf("ERRO %d\n", id);
        return false;
    }
}


int main(int argc, char** argv) {
    int id = atoi(argv[1]);
    int N = 3;

    link_t server = ufr_server_st("@new socket @coder msgpack @port %d @log 4", id+7000);


    for (int i=0; i<N; i++) {
        processos[i] = ufr_client("@new socket @coder msgpack @port %d @log 4", 7000+i);
    }


    link_t timer = ufr_subscriber("@new timer @time 2s");

    while ( ufr_loop() ) {
        if ( ufr_accept(&server) ) {
            int code;
            char comando[1024];
            const int res = ufr_get(&server, "> %d\n", &code);
            if ( code == 1 ) {
                ufr_put(&server, "%d\n\n", 99);
            }
        }

        if ( ufr_recv_async(&timer) ) {
            for (int i=0; i<N; i++) {
                if ( i == id ) continue;
                is_process_ok(i);
            }
        }


        /*if ( id == 1 ) {
            if ( ufr_recv_async(&timer) ) {
                teste_processo((id + 1) % 3 );
            }
        } else if ( id == 2 ) {
            if ( ufr_recv_async(&timer) ) {
                teste_processo((id + 1) % 3 );
            }
        }*/

        // printf("loop\n");
    }


    printf("FIM\n");
    // ufr_close(&server);
}