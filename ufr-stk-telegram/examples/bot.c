#include <ufr.h>
#include <stdio.h>

int ufr_gtw_telegram_new(link_t* link, int type);

int main() {
    const char* token = "8156512523:AAHdvmUYzxnTAOb3IvsItDjcqdRoDeVXHSw";
    link_t bot = ufr_server_st("@new %p @token %s", ufr_gtw_telegram_new, token);

    for (int i=0; i<5; i++) {
        char message[1024];
        ufr_get(&bot, "^s", message);
        printf("%s\n", message);
        ufr_put(&bot, "ssi\n", "aqui eh", "lucas", 20);
        ufr_put(&bot, "ssf\n", "aqui eh", "pedro", 20.2);
        ufr_put_eof(&bot);
    }

    ufr_close(&bot);
    
    return 0;
}


