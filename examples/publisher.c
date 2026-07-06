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
 * DISCLAIMED. IN NO EVENT SHALL aTHE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
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
#include <pthread.h>

// ============================================================================
//  Main
// ============================================================================

int main() {
    // abre um publicador
    // link_t pub = ufr_publisher("@new zmq @coder msgpack @log 5 @port 5000");
    // link_t pub = ufr_publisher("@new mqtt @coder text @log 5 @host 185.159.82.136 @topic intercampi");
    // link_t pub = ufr_publisher("@new posix:file @coder csv @log 5 @path saida.txt");
    // link_t pub = ufr_publisher("@new ros2 @coder ros2:tf @frame teste1 @child aaa");
    // link_t pub = ufr_publisher("@new webots @topic cmd_vel");

    // link_t* pub = ufr_publisher("@new mqtt @coder msgpack @topic teste @host 177.153.62.174");
    // link_t* pub = ufr_publisher("@new ros2 @coder ros2:string @topic teste @log 5");
    // link_t* pub = ufr_publisher("@new ros2 @coder ros2:twist @topic teste @log 5");

    ufr_stdout("@new posix:stdout @coder text @log 5 @host 177.153.62.174 @topic teste");

    // loop principal
    float vetor[10] = {1.0, 2.0, 3.0, 4.6, 5.6, 6.2, 7.8, 8.1, 9.0, 10.0};
    while( ufr_loop_ok() ) {
        ufr_printf("teste: %f aaa: %f vetor: %af  %af\n", 0.5, 0.2, 10, vetor);
        sleep(1);
    }

    // fim
    return 0;
}
