#include <ufr.h>
#include <stdio.h>

int ufr_gtw_telegram_new(link_t* link, int type);

int main() {
    const char* token = "8156512523:AAHdvmUYzxnTAOb3IvsItDjcqdRoDeVXHSw";
    // link_t bot = ufr_server_st("@new %p @token %s", ufr_gtw_telegram_new, token);
    link_t bot = ufr_server_st("@new telegram @token %s @log 5", token);

    // Loop principal
    while ( ufr_loop() ) {
        char message[1024];
        ufr_get(&bot, "> %s", message);
        printf("%s\n", message);
        ufr_put_eof(&bot);

        // ufr_put(&bot, "%s\n\n", "Fim");

        /*
        ufr_put(&bot, "%s\n", "Digite um número");
        ufr_get(&bot, "> %s", message);
        printf("%s\n", message);

        ufr_put(&bot, "%s\n", "Digite um novo número");
        ufr_get(&bot, "> %s", message);
        printf("%s\n", message);

        ufr_put(&bot, "%s\n", "A Soma deu 20");

        ufr_put_eof(&bot);
        */
    }

    // fim
    ufr_close(&bot);
    return 0;
}


