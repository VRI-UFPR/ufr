/* BSD 2-Clause License
 * 
 * Copyright (c) 2026, Visao Robotica e Imagem (VRI)
 *  - Samantha Vanessa Golim Stocco
 *  - Amaya Duarte Fagundes 
 *  - Felipe Gustavo Bombardelli
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

#include "ufr.h"
#include "ufr_test.h"

// ============================================================================
//  Tests 
// ============================================================================

void test1() {
    bool res;
    int cursor = 0;
    Evento evento;

    // Evento "%d"
    UFR_TEST_TRUE( ufr_parse_frase("%d %f     %s\n", &cursor, &evento) );
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_VAR_SCALAR ); 
    UFR_TEST_EQUAL_I32( evento.var, TIPO_I32 );   

    // Evento "%f"
    UFR_TEST_TRUE( ufr_parse_frase("%d %f     %s\n", &cursor, &evento) );
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_VAR_SCALAR );
    UFR_TEST_EQUAL_I32( evento.var, TIPO_F32 );

    // Evento "%s"
    UFR_TEST_TRUE( ufr_parse_frase("%d %f     %s\n", &cursor, &evento) );
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_VAR_SCALAR );
    UFR_TEST_EQUAL_I32( evento.var, TIPO_STR );

    // Evento "\n"
    UFR_TEST_TRUE( ufr_parse_frase("%d %f     %s\n", &cursor, &evento) );
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_SEND );

    // Fim, retorna 
    UFR_TEST_FALSE( ufr_parse_frase("%d %f     %s\n", &cursor, &evento) );
}

void test2() {
    Evento evento;
    int cursor = 0;

    UFR_TEST_TRUE( ufr_parse_frase("> nome= %s  idade= %d", &cursor, &evento) );
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_RECV1 );
    UFR_TEST_EQUAL_I32( evento.var, TIPO_NULO );

    UFR_TEST_TRUE( ufr_parse_frase("> nome= %s  idade= %d", &cursor, &evento) );
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_SEEK );
    UFR_TEST_EQUAL_I32( evento.var, TIPO_NULO );

    UFR_TEST_TRUE( ufr_parse_frase("> nome= %s  idade= %d", &cursor, &evento) );
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_VAR_SCALAR );
    UFR_TEST_EQUAL_I32( evento.var, TIPO_STR );

    UFR_TEST_TRUE( ufr_parse_frase("> nome= %s  idade= %d", &cursor, &evento) );
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_SEEK );
    UFR_TEST_EQUAL_I32( evento.var, TIPO_NULO );

    UFR_TEST_TRUE( ufr_parse_frase("> nome= %s  idade= %d", &cursor, &evento) );
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_VAR_SCALAR );
    UFR_TEST_EQUAL_I32( evento.var, TIPO_I32 );
}

void test3() {
    Evento evento;
    int cursor = 0;

    ufr_parse_frase("vetor= %a:f:10", &cursor, &evento);
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_SEEK );
    UFR_TEST_EQUAL_I32( evento.var, TIPO_NULO );

    ufr_parse_frase("vetor= %a:f:10", &cursor, &evento);
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_VAR_ARRAY );
    UFR_TEST_EQUAL_I32( evento.var, TIPO_F32 );
    UFR_TEST_EQUAL_I32 ( evento.tamanho[0], 10 );
}

int test4() {
    Evento evento;
    int cursor = 0;

    ufr_parse_frase("%a:ld:?", &cursor, &evento);
    UFR_TEST_EQUAL_I32( evento.tipo, EVENTO_VAR_ARRAY );
    UFR_TEST_EQUAL_I32( evento.var, TIPO_I64 );
    UFR_TEST_EQUAL_I32 ( evento.tamanho[0], -1 );
}

int main() {
    test1();
    test2();
    test3();
    test4();
    return 0;
}