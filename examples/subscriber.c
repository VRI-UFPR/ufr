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

// ============================================================================
//  Main
// ============================================================================

int main() {
    // link_t* sub = ufr_subscriber("@new ros2 @topic /cmd_vel @coder ros2:twist");
    ufr_stdin("@new ros2 @topic /cmd_vel @coder ros2:twist");

    // Main loop
    char matriz[480][640];
    for (int i=0; i<3; i++) {
        float a,b,nbytes;
        ufr_scanf("> %f %f", &a, &b);
        ufr_scanf("name=%a:f:1024 ", &matriz, &rows, &cols);
        // printf("%f %f\n", a, b);
    }

    // end
    return 0;
}

/*
>>              (evento de receber a primeira mensagem do cliente)
 >              (evento de receber mensagem)
\n              (evento de enviar mensagem)

%c              (copia um inteiro de 8bits)
%hu %hd         (copia um inteiro de 16bits)
%u  %d  %f      (copia um inteiro de 32bits)
%lu %ld %lf     (copia um inteiro de 64bits)
%-              (descarta um valor)
%0              (coloca valor nulo no campo)

%a:c:10         (copia uma cadeia de 10 int8_t)
%a:c:?          (copia uma cadeia de X int8_t)
%s              (copia uma cadeia de 8bits terminado com \0 de tamanho maximo de 1023+1(\0))
%s:100          (copia uma cadeia de 8bits terminado com \0 de tamanho maximo de 99+1(\0))

%a:hu:10        (copia uma cadeia de 10 inteiros de 16bits)
%a:hu:?         (copia uma cadeia de X inteiros de 16bits)
%a:hd:10        (copia uma cadeia de 10 inteiros de 16bits)
%a:hd:?         (copia uma cadeia de X inteiros de 16bits)

%a:u:10         (copia uma cadeia de 10 uint32_t)
%a:u:?          (copia uma cadeia de X uint32_t)
%a:d:40         (copia uma cadeia de 40 int64_t)
%a:d:?          (copia uma cadeia de X int64_t)
%a:f:10         (copia uma cadeia de 10 floats)
%a:f:?          (copia uma cadeia de X floats)

%a:lu:23        (copia uma cadeia de 23 uint32_t)
%a:lu:?         (copia uma cadeia de X uint32_t)
%a:ld:10        (copia uma cadeia de 10 int64_t)
%a:ld:?         (copia uma cadeia de X int64_t)
%a:lf:10        (copia uma cadeia de 10 floats)
%a:lf:?         (copia uma cadeia de X floats)

%m:u:3:4        (copia uma matriz 3x4 de uint64_t)
%m:d:3:4        (copia uma matriz 3x4 de int32_t)
%m:f:3:4        (copia uma matriz 3x4 de floats)

%m:lu:3:4       (copia uma matriz 3x4 de uint64_t)
%m:ld:3:4       (copia uma matriz 3x4 de int64_t)
%m:lf:3:4       (copia uma matriz 3x4 de double)

*/
