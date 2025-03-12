#include <stdio.h>
#include <cv_mat.h>

int main() {
    // cv_mat_t mat = cv_mat_new(480, 640, CV_8UC1);
    cv_mat_t mat = cv_imread("image.jpg");
    cv_imshow("teste", &mat);
    cv_waitkey(0);
    cv_mat_free(&mat);
    return 0;
}
