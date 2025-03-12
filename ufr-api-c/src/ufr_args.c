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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "ufr_args.h"

void* ufr_linux_load_library(const char* type, const char* name, const char* classname);

// ============================================================================
//  UFR ARGS
// ============================================================================

bool ufr_args_flex_div(const char* text, uint16_t* cursor_ini, char* token, const uint16_t token_max, const char div) {
    uint8_t state = 0;
    uint16_t i_token = 0;
    uint16_t i_text = *cursor_ini;
    token[0] = '\0';
    while (1) {
        const char c = text[i_text];
        if ( c == '\0' ) {
            token[i_token] = '\0';
            break;
        }
        
        // ignore caracter
        if ( c == '\n' ) {
            i_text += 1;
            continue;
        }

        // standard state
        if ( state == 0 ) {
        
            if ( c == '\'') {
                state = 1;

            } else if ( c == div ) {
                if ( i_token > 0 ) {
                    token[i_token] = '\0';
                    break;
                }

            } else {
                if ( i_token < token_max-1 ) {
                    token[i_token] = c;
                    i_token += 1;
                }
            }

        // inside quotes, example: 'text'
        } else if ( state == 1 ) {
            if ( c == '\'') {
                state = 0;
            } else {
                if ( i_token < token_max-1 ) {
                    token[i_token] = c;
                    i_token += 1;
                }
            }
        }

        i_text += 1;
    }

    *cursor_ini = i_text;
    return (i_token > 0);
}

// static
bool ufr_args_flex(const char* text, uint16_t* cursor_ini, char* token, const uint16_t token_max) {
    return ufr_args_flex_div(text, cursor_ini, token, token_max, ' ');
}

size_t ufr_args_getu(const ufr_args_t* args, const char* noun, const size_t default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( ufr_args_flex(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            ufr_args_flex(args->text, &cursor, token, sizeof(token));
            if ( token[0] == '%' ) {
                if ( token[1] == 'd' ) {
                    return args->arg[count_arg].i32;
                } else if ( token[1] == 's' ) {
                    return atoi(args->arg[count_arg].str);
                } else if ( token[1] == 'f' ) {
                    return (size_t) args->arg[count_arg].f32;
                }
            } else {
                return atoi(token);
            }
        }
    }

    // not found, return default value
    return default_value;
}

int ufr_args_geti(const ufr_args_t* args, const char* noun, const int default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( ufr_args_flex(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            ufr_args_flex(args->text, &cursor, token, sizeof(token));
            if ( token[0] == '%' ) {
                if ( token[1] == 'd' ) {
                    return args->arg[count_arg].i32;
                } else if ( token[1] == 's' ) {
                    return atoi(args->arg[count_arg].str);
                } else if ( token[1] == 'f' ) {
                    return (int) args->arg[count_arg].f32;
                }
            } else {
                return atoi(token);
            }
        }
    }

    // not found, return default value
    return default_value;
}

float ufr_args_getf(const ufr_args_t* args, const char* noun, const float default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( ufr_args_flex(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            ufr_args_flex(args->text, &cursor, token, sizeof(token));
            if ( token[0] == '%' ) {
                if ( token[1] == 'd' ) {
                    return args->arg[count_arg].i32;
                } else if ( token[1] == 's' ) {
                    return (float) atof(args->arg[count_arg].str);
                } else if ( token[1] == 'f' ) {
                    return args->arg[count_arg].f32;
                }
            } else {
                return (float) atof(token);
            }
        }
    }

    // not found, return default value
    return default_value;
}

const void* ufr_args_getp(const ufr_args_t* args, const char* noun, const void* default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( ufr_args_flex(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            ufr_args_flex(args->text, &cursor, token, sizeof(token));
            if ( token[0] == '%' ) {
                if ( token[1] == 'p' ) {
                    return args->arg[count_arg].ptr;
                } else {
                    return default_value;
                }
            }
        }
    }

    // not found, return default value
    return default_value;
}

const char* ufr_args_gets(const ufr_args_t* args, char* buffer, const char* noun, const char* default_value) {
    char token[UFR_ARGS_TOKEN];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( ufr_args_flex(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            ufr_args_flex(args->text, &cursor, token, sizeof(token));
            if ( token[0] == '%' ) {
                if ( token[1] == 's' ) {
                    return args->arg[count_arg].str;
                } else {
                    return default_value;
                }
            } else {
                strcpy(buffer, token);
                return buffer;
            }
        }
    }

    // not found, return default value
    return default_value;
}

void* ufr_args_getfunc(const ufr_args_t* args, const char* type, const char* noun, void* default_value) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;
    while( ufr_args_flex(args->text, &cursor, token, sizeof(token)) ) {
        // jump case word is not noun
        if ( token[0] != '@' ) {
            if ( token[0] == '%' ) {
                count_arg += 1;
            }
            continue;
        }

        // check if the noun is correct
        if ( strcmp(noun, token) == 0 ) {
            ufr_args_flex(args->text, &cursor, token, sizeof(token));
            if ( token[0] == '%' ) {
                if ( token[1] == 'p' ) {
                    return args->arg[count_arg].ptr;
                } else {
                    return default_value;
                }
            } else {
#if __linux__
/*
                char dl_name[512];
                char dl_class[512];
                uint16_t dl_cursor = 0;
                ufr_args_flex_div(token, &dl_cursor, dl_name, sizeof(dl_name), ':');
                ufr_args_flex_div(token, &dl_cursor, dl_class, sizeof(dl_class), ':');
                return ufr_linux_load_library(type, dl_name, dl_class);
*/
                return NULL;
#else
                return NULL;
#endif
            }
        }
    }

    // not found, return default value
    return default_value;
}

void ufr_args_load_from_va(ufr_args_t* args, const char* text, va_list list) {
    char token[512];
    uint8_t  count_arg = 0;
    uint16_t cursor = 0;

    // for each word in the text
    while( ufr_args_flex(text, &cursor, token, sizeof(token)) ) {
        if ( token[0] == '%' && token[1] == 'p' ) {               
            void* ptr = va_arg(list, void*);
            args->arg[ count_arg ].ptr = ptr;
            count_arg += 1;
        }
    }

    // success
    args->text = text;
}

// Mover essa funcao para UFR
// @new aaa @param1 bbb @@new ccc @@param1 ddd -> @new ccc @param1 ddd
int ufr_args_decrease_level(const char* src, char* dst) {
    dst[0] = '\0';
    char token[512];
    uint8_t count_arg = 0;
    uint16_t cursor = 0;
    bool ignore = true;
    while( ufr_args_flex(src, &cursor, token, sizeof(token)) ) {
        // uint32_t len = strlen(token);
        // if ( len > 2 ) {
        if ( token[0] == '@' ) {
            if ( token[1] == '@' ) {
                strcat(dst, &token[1]);
                strcat(dst, " ");
                ignore = false;
            } else {
                ignore = true;
            }
        } else {
            if ( ignore == false ) {
                strcat(dst, token);
                strcat(dst, " ");
            }
        }

    }

    return UFR_OK;
}
