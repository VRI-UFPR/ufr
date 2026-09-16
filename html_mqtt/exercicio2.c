#include <stdio.h>
#include <ufr.h>

int main() {
    link_t* link = ufr_subscriber("@new mqtt @coder msgpack @host klaso.cc @topic teste");
    for (int i=0; i<10; i++) {
        int a, b;
        ufr_get(link, "> %d %d\n", &a, &b);
        printf("%d %d\n", a, b);
    }
    return 0;
}

