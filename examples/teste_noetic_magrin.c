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

// ============================================================================
//  Main
// ============================================================================

int main() {
    link_t left = ufr_subscriber("@new ros_noetic @coder ros_noetic:i16 @topic left_encoder");
    link_t right = ufr_subscriber("@new ros_noetic @coder ros_noetic:i16 @topic right_encoder");
    link_t scan = ufr_subscriber("@new ros_noetic @coder ros_noetic:laserscan @topic scan");
    link_t us1 = ufr_subscriber("@new ros_noetic @coder ros_noetic:range @topic us1_urm");

    link_t imu = ufr_subscriber("@new ros_noetic @coder ros_noetic:imu @topic imu/data");
    link_t gps = ufr_subscriber("@new ros_noetic @coder ros_noetic:navsatfix @topic gps_neo8m");

    link_t rgb = ufr_subscriber("@new ros_noetic @coder ros_noetic:image @topic /camera/rgb/image_color");
    link_t depth = ufr_subscriber("@new ros_noetic @coder ros_noetic:image @topic /camera/depth/image");

    int left_val, right_val;
    float a,b,c,d,e,f;
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
            int status, service;
            float lat, log, alt;
            ufr_get(&gps, "iifff", &status, &service, &lat, &log, &alt);
            printf("gps %d %d %f %f %f\n", status, service, lat, log, alt);
        }

        if ( ufr_recv_async(&rgb) == UFR_OK ) {
            int height, width, step;
            ufr_get(&rgb, "iii", &height, &width, &step);
            printf("rgb %d %d %d\n", height, width, step);
            // salvar em jpeg
        }

        if ( ufr_recv_async(&depth) == UFR_OK ) {
            int height, width, step;
            ufr_get(&depth, "iii", &height, &width, &step);
            printf("depth %d %d %d\n", height, width, step);
            // salvar em jpeg
        }

        if ( ufr_recv_async(&imu) == UFR_OK ) {
            float x,y,z,w;
            ufr_get(&imu, "ffff", &x,&y,&z,&w);
            printf("imu %f %f %f %f\n", x, y, z, w);
            // salvar em jpeg
        }

        if ( ufr_recv_async(&left) == UFR_OK ) {
            ufr_get(&left, "i", &left_val);
            // printf("left %d\n", left_val);
        }

        if ( ufr_recv_async(&right) == UFR_OK ) {
            ufr_get(&right, "i", &right_val);
            // printf("right %d\n", right_val);
        }

        if ( ufr_recv_async(&scan) == UFR_OK ) {
            ufr_get(&scan, "ffffff", &a,&b,&c,&d,&e,&f);
            // printf("scan %f %f %f %f %f %f\n", a,b,c,d,e,f);
        }

        if ( ufr_recv_async(&us1) == UFR_OK ) {
            int type;
            ufr_get(&us1, "iffff", &type, &a, &b, &c, &d);
            // printf("us1 %d %f %f %f %f\n", type, a, b, c, d);
        }
    }

    ufr_close(&scan);
    ufr_close(&left);
    ufr_close(&right);
    ufr_close(&gps);
    ufr_close(&imu);
    return 0;
}
