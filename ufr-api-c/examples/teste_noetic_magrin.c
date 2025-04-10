/* BSD 2-Clause License
 * 
 * Copyright (c) 2024, Visao Robotica e Imagem (VRI)
 *  - Felipe Bombardelli <felipebombardelli@gmail.com>
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <ufr.h>
#include <unistd.h>
#include <time.h>

#include "cv_mat.h"

// ============================================================================
//  Main
// ============================================================================

int main() {
    link_t left = ufr_subscriber("@new ros_noetic @coder ros_noetic:i16 @topic left_encoder");
    link_t right = ufr_subscriber("@new ros_noetic @coder ros_noetic:i16 @topic right_encoder");
    link_t scan = ufr_subscriber("@new ros_noetic @coder ros_noetic:laserscan @topic scan");
    link_t us1 = ufr_subscriber("@new ros_noetic @coder ros_noetic:range @topic us1_urm");

    // link_t imu = ufr_subscriber("@new ros_noetic @coder ros_noetic:imu @topic imu/data");
    link_t gps = ufr_subscriber("@new ros_noetic @coder ros_noetic:navsatfix @topic gps_neo8m");

    link_t rgb = ufr_subscriber("@new ros_noetic @coder ros_noetic:image @topic camera_rgb");
    link_t depth = ufr_subscriber("@new ros_noetic @coder ros_noetic:image @topic camera_depth");

    // link_t saida = ufr_publisher("@new posix:file @coder csv @file saida.csv");

    int left_val, right_val;
    float angle_min, angle_max, angle_inc, time_inc, scan_time, range_min, range_max;
    float ranges[1024];
    float intensities[1024];
    int rgb_i = 0;
    int depth_i = 0;
    while ( ufr_loop_ok() ) {
        // ufr_get(&left, "^i", &left_val);
        // printf("%d\n", left_val);

        /*if ( ufr_recv_sy2(&left, &right, 100) == UFR_OK ) {
            ufr_get(&left, "i", left_val);
            ufr_get(&right, "i", right_val);
            // printf("%d %d\n", left_val, right_val);
        } else {
            printf("opa\n");
        }*/

        if ( ufr_recv_async(&gps) == UFR_OK ) {
            const clock_t stamp = clock();
            int status, service;
            float lat, log, alt;
            ufr_get(&gps, "iifff", &status, &service, &lat, &log, &alt);
            // printf("gps %d %d %f %f %f\n", status, service, lat, log, alt);
            printf("%ld, gps, %f, %f\n", stamp, lat, log);
        }


        if ( ufr_recv_async(&rgb) == UFR_OK ) {
            const clock_t stamp = clock();
            int height, width, step;
            ufr_get(&rgb, "iii", &height, &width, &step);
            void* data = ufr_get_rawptr(&rgb);

            /*cv_mat_t mat, bgr;
            cv_mat_init_with_ptr(&mat, height, width, CV_8UC3, data);
            cv_mat_init(&bgr,0,0,CV_8UC3);
            cv_cvtColor(&mat, &bgr, 1);
            cv_imshow("teste", &mat);
            cv_waitkey(1);
            cv_imwrite(filename, &bgr);
            cv_mat_free(&mat);*/

            rgb_i += 1;
            char filename[256];
            snprintf(filename, 256, "camera_rgb/%04d.png", rgb_i);
            printf("%ld, camera_rgb, %s\n", stamp, filename);
        }

        if ( ufr_recv_async(&depth) == UFR_OK ) {
            const clock_t stamp = clock();
            depth_i += 1;
            int height, width, step;
            ufr_get(&depth, "iii", &height, &width, &step);
            void* data = ufr_get_rawptr(&depth);

            char filename[256];
            snprintf(filename, 256, "camera_depth/%04d.tiff", depth_i);
            printf("%ld, camera_depth, %s\n", stamp, filename);
            
            /*cv_mat_t mat;
            cv_mat_init_with_ptr(&mat, height, width, CV_16U, data);
            cv_imwrite(filename, &mat);
            cv_mat_free(&mat);*/
        }

        /*if ( ufr_recv_async(&imu) == UFR_OK ) {
            float x,y,z,w;
            ufr_get(&imu, "ffff", &x,&y,&z,&w);
            printf("imu, %f, %f, %f, %f\n", x, y, z, w);
            // salvar em jpeg
        }*/ 


        if ( ufr_recv_async(&left) == UFR_OK ) {
            const clock_t stamp = clock();
            ufr_get(&left, "i", &left_val);
            printf("%ld, left, %d\n", stamp, left_val);
        }

        if ( ufr_recv_async(&right) == UFR_OK ) {
            const clock_t stamp = clock();
            ufr_get(&right, "i", &right_val);
            printf("%ld, right, %d\n", stamp, right_val);
        }

        if ( ufr_recv_async(&scan) == UFR_OK ) {
            const clock_t stamp = clock();
            ufr_get(&scan, "fffffff", &angle_min, &angle_max, &angle_inc, 
                &time_inc, &scan_time, &range_min, &range_max);
            printf("%ld, scan, %f, %f, %f, %f, %f, %f, %f, ", stamp, angle_min, angle_max, angle_inc, 
                time_inc, scan_time, range_min, range_max);

            const int size = ufr_get_af32(&scan, ranges, 1024);
            printf("array, %d, ", size);
            for (int i=0; i<size; i++) {
                printf("%f,", ranges[i]);
            }
            printf("\n");
        }


        if ( ufr_recv_async(&us1) == UFR_OK ) {
            const clock_t stamp = clock();
            int type;
            float a,b,c,d;
            ufr_get(&us1, "iffff", &type, &a, &b, &c, &d);
            printf("%ld, us1, %d, %f, %f, %f, %f\n", stamp, type, a, b, c, d);
        }


    }

    ufr_close(&us1);
    ufr_close(&rgb);
    ufr_close(&depth);
    ufr_close(&scan);
    ufr_close(&left);
    ufr_close(&right);
    // ufr_close(&gps);
    // ufr_close(&imu);
    return 0;
}
