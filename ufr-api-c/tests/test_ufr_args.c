/* BSD 2-Clause License
 * 
 * Copyright (c) 2024, Visao Robotica e Imagem (VRI)
 *  - Felipe Bombardelli <felipebombardelli@gmail.com>
 *  - Dayane O. de Carvalho <dayaneoliveira.eng@gmail.com>
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

// bool ufr_args_flex_div(const char* text, uint16_t* cursor_ini, char* token, const uint16_t token_max, const char div) {

void test1() {
    char token[32];
    uint16_t cursor = 0; 
    const char* text = "@nome valor";
    
    UFR_TEST_TRUE( ufr_args_flex_div(text, &cursor, token, 32, ' ') );
    UFR_TEST_EQUAL_STR(token, "@nome");
    UFR_TEST_TRUE( ufr_args_flex_div(text, &cursor, token, 32, ' ') );
    UFR_TEST_EQUAL_STR(token, "valor");
    UFR_TEST_FALSE( ufr_args_flex_div(text, &cursor, token, 32, ' ') );
    UFR_TEST_EQUAL_STR(token, "");
}

void test2() {
    char token[32];
    uint16_t cursor = 0; 

    // Tem erro neste caso!!!!
    // const char* text = "@nome 'valor composto'\n\n@host 10.0.0.1";

    const char* text = "@nome 'valor composto'\n\n @host 10.0.0.1";
    
    UFR_TEST_TRUE( ufr_args_flex_div(text, &cursor, token, 32, ' ') );
    UFR_TEST_EQUAL_STR(token, "@nome");
    UFR_TEST_TRUE( ufr_args_flex_div(text, &cursor, token, 32, ' ') );
    UFR_TEST_EQUAL_STR(token, "valor composto");
    UFR_TEST_TRUE( ufr_args_flex_div(text, &cursor, token, 32, ' ') );
    UFR_TEST_EQUAL_STR(token, "@host");
    UFR_TEST_TRUE( ufr_args_flex_div(text, &cursor, token, 32, ' ') );
    UFR_TEST_EQUAL_STR(token, "10.0.0.1");
    UFR_TEST_FALSE( ufr_args_flex_div(text, &cursor, token, 32, ' ') );
    UFR_TEST_EQUAL_STR(token, "");
}

void test3() {
    char token[32];
    uint16_t cursor = 0; 
    const char* text = "@nome 'valor composto'";
    
    UFR_TEST_TRUE( ufr_args_flex(text, &cursor, token, 32) );
    UFR_TEST_EQUAL_STR(token, "@nome");
    UFR_TEST_TRUE( ufr_args_flex(text, &cursor, token, 32) );
    UFR_TEST_EQUAL_STR(token, "valor composto");
    UFR_TEST_FALSE( ufr_args_flex(text, &cursor, token, 32) );
    UFR_TEST_EQUAL_STR(token, "");
}

void test4() {
    ufr_args_t args;
    char* nome = "fulano";
    char* text = "@nome %s @idade %d @altura %f @ponteiro %p";
    ufr_args_load_from(&args, text, nome, 20, 1.65, nome);
    UFR_TEST_EQUAL_PTR(args.text, text);
    UFR_TEST_EQUAL_PTR(args.arg[0].str, nome);
    UFR_TEST_EQUAL_I32(args.arg[1].i32, 20);
    UFR_TEST_EQUAL_F32(args.arg[2].f32, 1.65F);
    UFR_TEST_EQUAL_PTR(args.arg[3].ptr, nome);
}

void test5() {
    char nome[32];
    ufr_args_t args;
    ufr_args_load_from(&args, "@nome fulano @minimo 0 @maximo 80000000");
    UFR_TEST_EQUAL_STR( ufr_args_gets(&args, nome, "@nome", "desconhecido"), "fulano" );
    UFR_TEST_EQUAL_U64( ufr_args_getu(&args, "@minimo", 100), 0 );
    UFR_TEST_EQUAL_U64( ufr_args_getu(&args, "@maximo", 0), 80000000 );
    
    ufr_args_load_from(&args, "@nome %s @minimo %d @maximo %d", "fulano", -100, 100);
    UFR_TEST_EQUAL_STR( ufr_args_gets(&args, nome, "@nome", "desconhecido"), "fulano" );
    UFR_TEST_EQUAL_U64( ufr_args_getu(&args, "@minimo", 0), -100 );
    UFR_TEST_EQUAL_U64( ufr_args_getu(&args, "@maximo", 0), 100 );

    ufr_args_load_from(&args, "@minimo %s @maximo %f @medio %d", "-100", 100.25, 20);
    UFR_TEST_EQUAL_U64( ufr_args_getu(&args, "@minimo", 0), -100 );
    UFR_TEST_EQUAL_U64( ufr_args_geti(&args, "@medio", 0), 20 );
    UFR_TEST_EQUAL_U64( ufr_args_getu(&args, "@maximo", 0), 100 );
    UFR_TEST_EQUAL_U64( ufr_args_getu(&args, "@desconhecido", 30), 30 );
}

void test6() {
    ufr_args_t args;
    ufr_args_load_from(&args, "@minimo 0 @maximo 80000000");
    UFR_TEST_EQUAL_U64( ufr_args_geti(&args, "@minimo", 0), 0 );
    UFR_TEST_EQUAL_U64( ufr_args_geti(&args, "@maximo", 0), 80000000 );

    ufr_args_load_from(&args, "@minimo %s @maximo %f @medio %d", "-100", 100.25, 20);
    UFR_TEST_EQUAL_I64( ufr_args_geti(&args, "@minimo", 0), -100 );
    UFR_TEST_EQUAL_I64( ufr_args_geti(&args, "@medio", 0), 20 );
    UFR_TEST_EQUAL_I64( ufr_args_geti(&args, "@maximo", 0), 100 );
    UFR_TEST_EQUAL_I64( ufr_args_geti(&args, "@desconhecido", 30), 30 );
}

void test7() {
    ufr_args_t args;
    ufr_args_load_from(&args, "@minimo 1.250 @maximo 100.575");
    UFR_TEST_EQUAL_F64( ufr_args_getf(&args, "@minimo", 0), 1.250F );
    UFR_TEST_EQUAL_F64( ufr_args_getf(&args, "@maximo", 0), 100.575F );

    ufr_args_load_from(&args, "@minimo %s @maximo %f @medio %d", "-100", 100.25, 20);
    UFR_TEST_EQUAL_F64( ufr_args_getf(&args, "@minimo", 0), -100.00F );
    UFR_TEST_EQUAL_F64( ufr_args_getf(&args, "@medio", 0), 20.00F );
    UFR_TEST_EQUAL_F64( ufr_args_getf(&args, "@maximo", 0), 100.25F );
    UFR_TEST_EQUAL_F64( ufr_args_getf(&args, "@desconhecido", 30.0), 30.0 );
}

void test8() {
    char nome[32];
    ufr_args_t args;
    ufr_args_load_from(&args, "@ponteiro %p @outro %d", nome, 20);
    UFR_TEST_EQUAL_PTR( ufr_args_getp(&args, "@ponteiro", NULL), nome );
    UFR_TEST_NULL( ufr_args_getp(&args, "@desconhecido", NULL) );
    UFR_TEST_NULL( ufr_args_getp(&args, "@outro", NULL) );
}

void test9() {
    char nome[32];
    char buffer[32];
    ufr_args_t args;
    ufr_args_load_from(&args, "@ponteiro %p @inteiro %d @ponto %f @string %s @direto opa @nulo %s", nome, 20, 30.255, "teste", NULL);
    UFR_TEST_EQUAL_STR( ufr_args_gets(&args, buffer, "@ponteiro", ""), "" );
    UFR_TEST_NULL( ufr_args_gets(&args, buffer, "@nulo", "") ); 
        UFR_TEST_EQUAL_STR( buffer, "" );
    UFR_TEST_EQUAL_STR( ufr_args_gets(&args, buffer, "@desconhecido", ""), "" );
}

void test10() {
    char output[128];
    ufr_args_decrease_level("@param 11 @@new mqtt @@coder msgpack", output);
    UFR_TEST_EQUAL_STR(output, "@new mqtt @coder msgpack ");
    // corrigir espaço final da string
}

void test11() {
    // ufr_args_t args;
    // ufr_args_load_from(&args, "@outro %d  @funcao %p", test11, 10);
    // int (*function_ptr)() = ufr_args_getfunc(&args, "gtw", "@funcao", NULL);
    // ERROR
    // UFR_TEST_EQUAL_PTR(function_ptr, (void*) test11);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test1();
    test2();
    test3();
    test4();
    test5();
    test6();
    test7();
    test8();
    test9();
    test10();
    test11();
    return 0;
}


