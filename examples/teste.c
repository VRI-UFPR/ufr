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
    // link_t left = ufr_subscriber("@new ros_melodic @coder ros_melodic:i16 @topic left_encoder");
    // link_t right = ufr_subscriber("@new ros_melodic @coder ros_melodic:i16 @topic right_encoder");
    link_t scan = ufr_subscriber("@new ros_melodic @coder ros_melodic:laserscan @topic scan");

    int left_val, right_val;
    float a,b,c,d,e,f;
    while ( ufr_loop_ok() ) {
        /*if ( ufr_recv_sy2(&left, &right, 100) == UFR_OK ) {
            ufr_get(&left, "i", left_val);
            ufr_get(&right, "i", right_val);
            // printf("%d %d\n", left_val, right_val);
        } else {
            printf("opa\n");
        }*/

        if ( ufr_recv_async(&scan) == UFR_OK ) {
            ufr_get(&scan, "ffffff", &a,&b,&c,&d,&e,&f);
            printf("opa %f %f %f %f %f %f\n", a,b,c,d,e,f);
        }
    }

    ufr_close(&scan);
    // ufr_close(&left);
    // ufr_close(&right);
    return 0;
}
