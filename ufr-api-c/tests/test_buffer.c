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

// Imprime estado do buffer.
void ufr_buffer_print (ufr_buffer_t* buffer) {

    printf ("BUFFER: SIZE = %ld, MAX = %ld, DATA = %.*s\n", buffer->size, buffer->max, (int)buffer->size, buffer->ptr);
}

// Criar um novo buffer.
void test_buffer_init () {
    
    printf ("Iniciando testes...\n");
    printf ("------------------------------------------------------------------------------");
    printf ("\n");

    ufr_buffer_t buffer;
    ufr_buffer_init (&buffer); // Inicializa o buffer.
    UFR_TEST_NOT_NULL (&buffer); /* Verifica se o ponteiro retornado não é NULL */
    UFR_TEST_EQUAL_U64 (buffer.size, 0); /* Verifica se o tamanho inicial do buffer é 0 */ 
    UFR_TEST_EQUAL_U64 (buffer.max, MESSAGE_ITEM_SIZE); /* Verifica se a capacidade máxima do
                                                           * buffer é igual a MESSAGE_ITEM_SIZE 
                                                           * (valor definido em ufr_buffer.h). */
    UFR_TEST_NOT_NULL (buffer.ptr); 
    printf ("Teste_buffer_init OK\n");
    printf ("\n");  
    
    ufr_test_print_result ();    
    printf ("------------------------------------------------------------------------------");
    printf ("\n");                                                    
}


void test_buffer_new () {
    
    ufr_buffer_t* buffer = ufr_buffer_new (); /* Cria um novo buffer dinamicamente.*/
    
    UFR_TEST_NOT_NULL (buffer); /* Verifica se o ponteiro retornado não é NULL */
    printf ("Test_buffer_new OK\n");
    
    ufr_buffer_free (buffer);
    UFR_TEST_ZERO (buffer->size);
    printf ("\n");
    
    ufr_test_print_result (); 
    printf ("------------------------------------------------------------------------------");
    printf ("\n");                          
}

// Limpa buffer.
void test_buffer_clear () {
   
    ufr_buffer_t* buffer = ufr_buffer_new ();
    
    UFR_TEST_NOT_NULL (buffer);
    UFR_TEST_EQUAL_U64 (buffer->size, 0);
    UFR_TEST_EQUAL_U64 (buffer->max, MESSAGE_ITEM_SIZE);

    printf ("         Test_buffer_clear\n");
    printf ("\n");
    printf ("Estado inicial do buffer:\n");
    ufr_buffer_print (buffer);
    printf ("\n");
    printf ("Inserindo 1 caractere...\n");
    
    ufr_buffer_put_chr (buffer, 'A'); 
    ufr_buffer_print (buffer);
    printf ("\n");
    
    ufr_buffer_put_chr (buffer, 'A'); 
    ufr_buffer_print (buffer);
    printf ("\n");
    
    ufr_buffer_clear (buffer);
    printf ("Estado do buffer apos clear...\n");
    ufr_buffer_print (buffer);
    
    ufr_buffer_free (buffer);
    UFR_TEST_ZERO (buffer->size); 
    printf ("\n");
    
    ufr_test_print_result ();
    printf ("------------------------------------------------------------------------------");
    printf ("\n");  
}

// Libera memória.
void test_buffer_free () {

    ufr_buffer_t* buffer = ufr_buffer_new ();
    
    UFR_TEST_NOT_NULL (buffer);
    UFR_TEST_EQUAL_U64 (buffer->size, 0);

    printf ("          Test_buffer_free\n");
    printf ("\n");
    ufr_buffer_free (buffer);
    printf ("Memoria Liberada\n");
    UFR_TEST_ZERO (buffer->size);
    printf ("\n");
    
    ufr_test_print_result ();
    printf ("------------------------------------------------------------------------------");
    printf ("\n"); 
}

// Testa redimensionamento.
void test_check_size () {

    ufr_buffer_t* buffer = ufr_buffer_new ();
    
    UFR_TEST_NOT_NULL (buffer);
    UFR_TEST_EQUAL_U64 (buffer->size, 0);
    UFR_TEST_EQUAL_U64 (buffer->max, MESSAGE_ITEM_SIZE);

    printf ("          Test_check_size\n");
    printf ("\n");
    printf ("1) Estado inicial do buffer:\n");
    ufr_buffer_print (buffer);
    printf ("\n");

    // Adiciona dados ao buffer.
    const char* data1 = "ABCDE";
    ufr_buffer_put (buffer, data1, strlen(data1));
    printf ("2) Adicionando dado...\n");
    printf ("Dado: %s\n", data1);
    ufr_buffer_print (buffer);
    printf ("\n");
    
    //Adiciona mais dados para testar redimensionamento.
    const char* data2 = "12345678910";
    ufr_buffer_put (buffer, data2, strlen(data2));
    printf ("3) Adicionando mais dados p/ testar redimensionamento...\n");
    printf ("Dado: %s\n", data2);
    ufr_buffer_print (buffer);
    
    ufr_buffer_free (buffer);
    UFR_TEST_ZERO (buffer->size);
    printf ("\n");
    
    ufr_test_print_result ();
    printf ("------------------------------------------------------------------------------");
    printf ("\n"); 
}

// Inserção de dados.
void test_buffer_put () {
    
    ufr_buffer_t* buffer = ufr_buffer_new ();
    
    UFR_TEST_NOT_NULL (buffer);
    UFR_TEST_EQUAL_U64 (buffer->size, 0);
    UFR_TEST_EQUAL_U64 (buffer->max, MESSAGE_ITEM_SIZE);
    
    printf ("          Test_buffer_put\n");
    printf ("\n");
    printf ("Estado inicial do buffer:\n");
    ufr_buffer_print (buffer);
    printf ("\n");
    printf ("Apos dados inseridos...\n");
    
    ufr_buffer_put (buffer, "Dado1", 5);
    UFR_TEST_EQUAL_U64 (buffer->size, 5);
    UFR_TEST_EQUAL_STR (buffer->ptr, "Dado1");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put (buffer, "Dado2", 5);
    UFR_TEST_EQUAL_U64 (buffer->size, 10);
    UFR_TEST_EQUAL_STR (buffer->ptr, "Dado1Dado2");
    ufr_buffer_print (buffer);

    const char* text3 = "hfjakfhjsjsdbhwsdfk,1254@";
    const int text_len3 = strlen(text3);
    ufr_buffer_put (buffer, text3, text_len3);
    UFR_TEST_EQUAL_U64 (buffer->size, 35);
    UFR_TEST_EQUAL_STR (buffer->ptr, "Dado1Dado2hfjakfhjsjsdbhwsdfk,1254@");
    
    ufr_buffer_print (buffer);

    ufr_buffer_free (buffer);
    UFR_TEST_ZERO (buffer->size);
    printf ("\n");
    
    ufr_test_print_result ();
    printf ("------------------------------------------------------------------------------");
    printf ("\n"); 
}

// Inserção de caractere.
void test_buffer_put_chr () {

    ufr_buffer_t* buffer = ufr_buffer_new ();

    UFR_TEST_NOT_NULL (buffer);
    UFR_TEST_EQUAL_U64 (buffer->size, 0);
    UFR_TEST_EQUAL_U64 (buffer->max, MESSAGE_ITEM_SIZE);
    
    printf ("          Test_buffer_put_chr\n");
    printf ("\n");
    printf ("Estado inicial do buffer:\n");
    ufr_buffer_print (buffer);
    printf ("\n");
    printf ("Inserindo caractere...\n");
    printf ("\n");
    
    ufr_buffer_put_chr (buffer, 'A');
    UFR_TEST_EQUAL_U64 (buffer->size, 1);
    UFR_TEST_EQUAL_STR (buffer->ptr, "A");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_chr (buffer, 'B');
    UFR_TEST_EQUAL_U64 (buffer->size, 2);
    UFR_TEST_EQUAL_STR (buffer->ptr, "AB");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_chr (buffer, '&');
    UFR_TEST_EQUAL_U64 (buffer->size, 3);
    UFR_TEST_EQUAL_STR (buffer->ptr, "AB&");
    ufr_buffer_print (buffer);

    ufr_buffer_free (buffer);
    UFR_TEST_ZERO (buffer->size);
    printf ("\n");

    ufr_test_print_result ();
    printf ("------------------------------------------------------------------------------");
    printf ("\n");
}

// Converte valor inteiro sem sinal de 8 bits em uma string.
void test_buffer_put_u8_as_str () {

    ufr_buffer_t* buffer = ufr_buffer_new ();
    
    UFR_TEST_NOT_NULL (buffer);
    UFR_TEST_EQUAL_U64 (buffer->size, 0);
    UFR_TEST_EQUAL_U64 (buffer->max, MESSAGE_ITEM_SIZE);

    // Valores no intervalo 0 a 255.
    printf ("          Test_buffer_put_u8_as_str (0 a 255)\n");
    printf ("\n");
    UFR_TEST_EQUAL_U64 (buffer->size, 0);
    UFR_TEST_EQUAL_U64 (buffer->max, MESSAGE_ITEM_SIZE);
    printf ("Estado inicial do buffer:\n");
    ufr_buffer_print (buffer);
    printf ("\n");
    printf ("Inserindo valor...\n");
    printf ("\n");
    
    ufr_buffer_put_u8_as_str (buffer, 13);
    UFR_TEST_EQUAL_U64 (buffer->size, 2);
    UFR_TEST_EQUAL_STR (buffer->ptr, "13");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_u8_as_str (buffer, 255);
    UFR_TEST_EQUAL_U64 (buffer->size, 6);
    UFR_TEST_EQUAL_STR (buffer->ptr, "13 255");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_u8_as_str (buffer, 1);
    UFR_TEST_EQUAL_U64 (buffer->size, 8);
    UFR_TEST_EQUAL_STR (buffer->ptr, "13 255 1");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_u8_as_str (buffer, -37);
    UFR_TEST_EQUAL_U64 (buffer->size, 12);
    UFR_TEST_EQUAL_STR (buffer->ptr, "13 255 1 219");
    ufr_buffer_print (buffer);
    
    ufr_buffer_free (buffer);
    UFR_TEST_ZERO (buffer->size);
    printf ("\n");

    ufr_test_print_result ();
    printf ("------------------------------------------------------------------------------");
    printf ("\n");
}

// Converte valor inteiro com sinal de 8 bits em uma string.
void test_buffer_put_i8_as_str () {

    ufr_buffer_t* buffer = ufr_buffer_new ();

    UFR_TEST_NOT_NULL (buffer);
    UFR_TEST_EQUAL_U64 (buffer->size, 0);
    UFR_TEST_EQUAL_U64 (buffer->max, MESSAGE_ITEM_SIZE);

    // Valores no intervalo -128 a 127.
    printf ("          Test_buffer_put_i8_as_str (-128 a 127)\n");
    printf ("\n");
    printf ("Estado inicial do buffer:\n");
    ufr_buffer_print (buffer);
    printf ("\n");
    printf ("Inserindo valor...\n");
    printf ("\n");
    
    ufr_buffer_put_i8_as_str (buffer, -128);
    UFR_TEST_EQUAL_U64 (buffer->size, 4);
    UFR_TEST_EQUAL_STR (buffer->ptr, "-128");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_i8_as_str (buffer, -100);
    UFR_TEST_EQUAL_U64 (buffer->size, 9);
    UFR_TEST_EQUAL_STR (buffer->ptr, "-128 -100");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_i8_as_str (buffer, 12);
    UFR_TEST_EQUAL_U64 (buffer->size, 12);
    UFR_TEST_EQUAL_STR (buffer->ptr, "-128 -100 12");
    ufr_buffer_print (buffer);
    
    ufr_buffer_free (buffer);
    UFR_TEST_ZERO (buffer->size);
    printf ("\n");

    ufr_test_print_result ();
    printf ("------------------------------------------------------------------------------");
    printf ("\n");
}

// Converte valor inteiro sem sinal de 32 bits em uma string.
void test_buffer_put_u32_as_str () {

    ufr_buffer_t* buffer = ufr_buffer_new ();

    UFR_TEST_NOT_NULL (buffer);
    UFR_TEST_EQUAL_U64 (buffer->size, 0);
    UFR_TEST_EQUAL_U64 (buffer->max, MESSAGE_ITEM_SIZE);

    // Valores no intervalo 0 a 2³² - 1 (0 a 4.294.967.295).
    printf ("          Test_buffer_put_u32_as_str (0 a 2³² - 1 (0 a 4.294.967.295))\n");
    printf ("\n");
    printf ("Estado inicial do buffer:\n");
    ufr_buffer_print (buffer);
    printf ("\n");
    printf ("Inserindo valor...\n");
    printf ("\n");
    
    ufr_buffer_put_u32_as_str (buffer, 1250);
    UFR_TEST_EQUAL_U64 (buffer->size, 4);
    UFR_TEST_EQUAL_STR (buffer->ptr, "1250");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_u32_as_str (buffer, 25);
    UFR_TEST_EQUAL_U64 (buffer->size, 7);
    UFR_TEST_EQUAL_STR (buffer->ptr, "1250 25");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_u32_as_str (buffer, 4294967295);
    UFR_TEST_EQUAL_U64 (buffer->size, 18);
    UFR_TEST_EQUAL_STR (buffer->ptr, "1250 25 4294967295");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_u32_as_str (buffer, -1);
    UFR_TEST_EQUAL_U64 (buffer->size, 29);
    UFR_TEST_EQUAL_STR (buffer->ptr, "1250 25 4294967295 4294967295");
    ufr_buffer_print (buffer);
    
    ufr_buffer_free (buffer);
    UFR_TEST_ZERO (buffer->size);
    printf ("\n");

    ufr_test_print_result ();
    printf ("------------------------------------------------------------------------------");
    printf ("\n");
}

// Converte valor inteiro com sinal de 32 bits em uma string.
void test_buffer_put_i32_as_str () {

    ufr_buffer_t* buffer = ufr_buffer_new ();

    UFR_TEST_NOT_NULL (buffer);
    UFR_TEST_EQUAL_U64 (buffer->size, 0);
    UFR_TEST_EQUAL_U64 (buffer->max, MESSAGE_ITEM_SIZE);

    // Valores no intervalo -2³¹ a 2³¹ - 1 (-2.147.483.648 a 2.147.483.647)
    printf ("          Test_buffer_put_i32_as_str (-2³¹ a 2³¹ - 1 (-2.147.483.648 a 2.147.483.647))\n");
    printf ("\n");
    printf ("Estado inicial do buffer:\n");
    ufr_buffer_print (buffer);
    printf ("\n");
    
    ufr_buffer_put_i32_as_str (buffer, -2147483600);
    UFR_TEST_EQUAL_U64 (buffer->size, 11);
    UFR_TEST_EQUAL_STR (buffer->ptr, "-2147483600");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_i32_as_str (buffer, -2147483648);
    UFR_TEST_EQUAL_U64 (buffer->size, 23);
    UFR_TEST_EQUAL_STR (buffer->ptr, "-2147483600 -2147483648");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_i32_as_str (buffer, 1350);
    UFR_TEST_EQUAL_U64 (buffer->size, 28);
    UFR_TEST_EQUAL_STR (buffer->ptr, "-2147483600 -2147483648 1350");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_i32_as_str (buffer, 2147483648);
    UFR_TEST_EQUAL_U64 (buffer->size, 40);
    UFR_TEST_EQUAL_STR (buffer->ptr, "-2147483600 -2147483648 1350 -2147483648");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_i32_as_str (buffer, 3000000000);
    UFR_TEST_EQUAL_U64 (buffer->size, 52);
    UFR_TEST_EQUAL_STR (buffer->ptr, "-2147483600 -2147483648 1350 -2147483648 -1294967296");
    ufr_buffer_print (buffer);
    
    ufr_buffer_free (buffer);
    UFR_TEST_ZERO (buffer->size);
    printf ("\n");

    ufr_test_print_result ();
    printf ("------------------------------------------------------------------------------");
    printf ("\n");
}

// Converte um valor de ponto flutuante de 32 bits em uma string.
void test_buffer_put_f32_as_str () {

    ufr_buffer_t* buffer = ufr_buffer_new ();

    UFR_TEST_NOT_NULL (buffer);
    UFR_TEST_EQUAL_U64 (buffer->size, 0);
    UFR_TEST_EQUAL_U64 (buffer->max, MESSAGE_ITEM_SIZE);

    // Valores no intervalo 1.2E-38 a 3.4E+38 (6 digitos)
    printf ("          Test_buffer_put_f32_as_str (1.2E-38 a 3.4E+38 (6 digitos))\n");
    printf ("\n");
    printf ("Estado inicial do buffer:\n");
    ufr_buffer_print (buffer);
    printf ("\n");
    
    ufr_buffer_put_f32_as_str (buffer, 0.000012F);
    UFR_TEST_EQUAL_U64 (buffer->size, 8);
    UFR_TEST_EQUAL_STR (buffer->ptr, "0.000012");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_f32_as_str (buffer, 0);
    UFR_TEST_EQUAL_U64 (buffer->size, 17);
    UFR_TEST_EQUAL_STR (buffer->ptr, "0.000012 0.000000");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_f32_as_str (buffer, 34.0000000000000000000000000000000000000);
    UFR_TEST_EQUAL_U64 (buffer->size, 27);
    UFR_TEST_EQUAL_STR (buffer->ptr, "0.000012 0.000000 34.000000");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_f32_as_str (buffer, -12.0123456F);
    UFR_TEST_EQUAL_U64 (buffer->size, 38);
    UFR_TEST_EQUAL_STR (buffer->ptr, "0.000012 0.000000 34.000000 -12.012345");
    ufr_buffer_print (buffer);
    
    ufr_buffer_free (buffer);
    UFR_TEST_ZERO (buffer->size);
    printf ("\n");

    ufr_test_print_result ();
    printf ("------------------------------------------------------------------------------");
    printf ("\n");

}

// Adiciona uma string (text) ao buffer.
void test_buffer_put_str () {

    ufr_buffer_t* buffer = ufr_buffer_new ();

    UFR_TEST_NOT_NULL (buffer);
    UFR_TEST_EQUAL_U64 (buffer->size, 0);
    UFR_TEST_EQUAL_U64 (buffer->max, MESSAGE_ITEM_SIZE);

    printf ("          Test_buffer_put_str\n");
    printf ("\n");
    printf ("Estado inicial do buffer:\n");
    ufr_buffer_print (buffer);
    printf ("\n");
    
    ufr_buffer_put_str (buffer, "teste 1");
    UFR_TEST_EQUAL_U64 (buffer->size, 7);
    UFR_TEST_EQUAL_STR (buffer->ptr, "teste 1");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_str (buffer, "teste 2");
    UFR_TEST_EQUAL_U64 (buffer->size, 15);
    UFR_TEST_EQUAL_STR (buffer->ptr, "teste 1 teste 2");
    ufr_buffer_print (buffer);
    
    ufr_buffer_put_str (buffer, "teste3 teste4 teste5 teste6 teste7 ");
    UFR_TEST_EQUAL_U64 (buffer->size, 51);
    UFR_TEST_EQUAL_STR (buffer->ptr, "teste 1 teste 2 teste3 teste4 teste5 teste6 teste7 ");
    ufr_buffer_print (buffer);


    ufr_buffer_free (buffer);
    UFR_TEST_ZERO (buffer->size);
    printf ("\n");

    ufr_test_print_result ();
    printf ("------------------------------------------------------------------------------");
    printf ("\n");
}

// Testa entradas nulas nas funções.
void test_entrada_nula() {
    ufr_buffer_init (NULL);
    ufr_buffer_clear (NULL);
    ufr_buffer_free(NULL);
    ufr_buffer_check_size(NULL, 10);
    ufr_buffer_put (NULL, "Dado", 4);
    ufr_buffer_put_chr (NULL, 'A');
    ufr_buffer_put_u8_as_str (NULL, 255);
    ufr_buffer_put_i8_as_str (NULL, -128);
    ufr_buffer_put_u32_as_str (NULL, 1250);
    ufr_buffer_put_i32_as_str (NULL, 1350);
    ufr_buffer_put_f32_as_str (NULL, 0);
    ufr_buffer_put_str (NULL, "teste 1");
}

int main() {

    printf ("\n");
    printf ("Testes de entrada nula...\n");
    printf ("\n");
    test_entrada_nula          ();
    test_buffer_init           (); 
    test_buffer_new            ();
    test_buffer_clear          ();
    test_buffer_free           ();
    test_check_size            (); 
    test_buffer_put            ();
    test_buffer_put_chr        ();
    test_buffer_put_u8_as_str  ();
    test_buffer_put_i8_as_str  ();
    test_buffer_put_u32_as_str ();
    test_buffer_put_i32_as_str ();
    test_buffer_put_f32_as_str ();
    test_buffer_put_str        ();
    
    return 0;
}