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

int ufr_exec(link_t* link, const char* command) {
    int code;
    ufr_put(link, "s\n\n", command);
    ufr_get(link, "^i", &code);
    return code;
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    link_t link = ufr_client("@new zmq @coder msgpack");
    ufr_exec(&link, "start");
    /*
        nome, text
        nome, text
        nome, text
    */

    link_t left = ufr_app_subscriber("left_encoder");

    /*link_t db = ufr_client("@new sqlite @file development.sqlite3 @table departments");
    int res = ufr_exec(&db, "select * from departments");
    ufr_close(&db);*/

    link_t motors = ufr_subscriber("@new ros_humble @coder ros_humble:twist @topic cmd_vel");
    while ( ufr_loop_ok() ) {
        float vel1, vel2;
        ufr_get(&motors, "^ff", &vel1, &vel2);
        printf("%f %f\n", vel1, vel2);
    }

    ufr_close(&motors);
    return 0;
}


/*


Shell de Base de Dados (python)
  - ls, echo, 

  HTTP e Shell
       |
       | ZMQ (Client)
       |
   IA da UFPR (recebe NL e converte para instrucoes) (python)
       |
       | ZMQ (login, socket)
       |   - comandos simples e fixos.  
       |   - crie departamento %s
       |   - crie usuario
       |   - 
       |
Aplicativo de Base de Dados (C)
       |
   PostgreSQL ou SQLite3



*/