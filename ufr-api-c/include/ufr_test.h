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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern int g_contador;

// ============================================================================
//  Macros
// ============================================================================

#define ufr_test_print_result() printf("OK - %d\n", g_contador)

#define UFR_TEST_EQUAL(current, expected) if ( current == expected ) { g_contador++; } else { printf("Error:%s:%d: the value is %d, but expected %d\n", __FILE__, __LINE__, current, expected ); exit(1); }

#define UFR_TEST_EQUAL_I32(current, expected) if ( current == expected ) { g_contador++; } else { printf("Error:%s:%d: the value is %d, but expected %d\n", __FILE__, __LINE__, (int32_t) current, (int32_t)expected ); exit(1); }

#define UFR_TEST_EQUAL_U32(current, expected) if ( current == expected ) { g_contador++; } else { printf("Error:%s:%d: the value is %u, but expected %u\n", __FILE__, __LINE__, (uint32_t) current, (uint32_t) expected ); exit(1); }

#define UFR_TEST_EQUAL_F32(current, expected) if ( current == expected ) {  g_contador++;} else { printf("Error:%s:%d: the value is %f, but expected %f\n", __FILE__, __LINE__, current, expected ); exit(1); }

#define UFR_TEST_EQUAL_I64(current, expected) if ( current == expected ) { g_contador++; } else { printf("Error:%s:%d: the value is %ld, but expected %ld\n", __FILE__, __LINE__, (int64_t) current, (int64_t) expected ); exit(1); }

#define UFR_TEST_EQUAL_U64(current, expected) if ( current == expected ) { g_contador++; } else { printf("Error:%s:%d: the value is %lu, but expected %lu\n", __FILE__, __LINE__, (uint64_t) current, (uint64_t) expected );  }

#define UFR_TEST_EQUAL_F64(current, expected) if ( current == expected ) { g_contador++; } else { printf("Error:%s:%d: the value is %g, but expected %g\n", __FILE__, __LINE__, (double) current, (double) expected ); printf("TESTE 1!"); exit(1); }

#define UFR_TEST_EQUAL_STR(current, expected) if ( strcmp(current, expected) == 0 ) { g_contador++; } else { printf("Error:%s:%d: the value is \"%s\", but expected \"%s\"\n", __FILE__, __LINE__, current, expected); exit(1); }

#define UFR_TEST_TRUE(current) if ( current ) { g_contador++; } else { printf("Error:%s:%d: the value is %ld, but expected != 0\n", __FILE__, __LINE__, (int64_t) current); exit(1); }

#define UFR_TEST_FALSE(current) if ( !current ) { g_contador++; } else { printf("Error:%s:%d: the value is %ld, but expected 0\n", __FILE__, __LINE__, (int64_t) current); exit(1); }

#define UFR_TEST_ZERO(current) if ( current == 0 ) { g_contador++; } else { printf("Error:%s:%d: the value is %ld, but expected 0\n", __FILE__, __LINE__, (int64_t) current); exit(1); }

#define UFR_TEST_NULL(current) if ( current == NULL ) { g_contador++; } else { printf("Error:%s:%d: the value is %p, but expected NULL\n", __FILE__, __LINE__, (void*) current); exit(1); }

#define UFR_TEST_NOT_NULL(current) if ( current != NULL ) { g_contador++; } else { printf("Error:%s:%d: the value is NULL, but expected not NULL\n", __FILE__, __LINE__); exit(1); }

#define UFR_TEST_OK(current) if ( current == UFR_OK ) { g_contador++; } else { printf("Error:%s:%d: the value is %d, but expected %d\n", __FILE__, __LINE__, (int) current, UFR_OK); exit(1); }



