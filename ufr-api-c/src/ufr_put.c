/* BSD 2-Clause License
 * 
 * Copyright (c) 2024, Visao Robotica e Imagem (VRI)
 *  - Felipe Bombardelli <felipebombardelli@gmail.com>
 *  - Samantha Vanessa Golim Stocco
 *  - Amaya 
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
#include "ufr.h"


int ufr_put_begin_package(link_t* link);


// ============================================================================
//  PUT
// ============================================================================

/**
 * @author Samantha Vanessa Golim Stocco
 * @author Amaya Duarte Fagundes
 */
bool ufr_parse_frase(const char* frase, int* inout_cursor, Evento* out_evento) {
    // Pular espaços em branco
    while (frase[*inout_cursor] == ' ') {
        (*inout_cursor)++;
    }

    // Fim da frase 
    if (frase[*inout_cursor] == '\0') {
        return false;
    }

    // Inicialização 
    out_evento->var = TIPO_NULO;
    out_evento->tamanho[0] = 0;
    out_evento->tamanho[1] = 0;
    out_evento->nome[0] = '\0';

    int pos = *inout_cursor;

    // Caso 1: (\n)
    if (frase[pos] == '\n') {
        out_evento->tipo = EVENTO_SEND;
        (*inout_cursor)++;
        return true;
    }

    // Caso 2: (> ou >>)
    if (frase[pos] == '>') {
        if (frase[pos + 1] == '>') {
            out_evento->tipo = EVENTO_RECV2;
            (*inout_cursor) += 2;
        } else {
            out_evento->tipo = EVENTO_RECV1;
            (*inout_cursor)++;
        }
        return true;
    }

    // Caso 3: (%)
    if (frase[pos] == '%') {
        out_evento->tipo = EVENTO_VAR;
        pos++; 

        // Verifica se é um Array (%a:)
        if (frase[pos] == 'a' && frase[pos + 1] == ':') {
            pos += 2; 
            
            if (frase[pos] == 'c') {
                out_evento->var = TIPO_ARRAY_C; pos++; 
            }
            else if (frase[pos] == 'f') {
                out_evento->var = TIPO_ARRAY_F32; pos++; 
            }
            else if (frase[pos] == 'u') {
                out_evento->var = TIPO_ARRAY_U32; pos++; 
            }
            else if (frase[pos] == 'd') { 
                out_evento->var = TIPO_ARRAY_I32; pos++; 
            }
            else if (frase[pos] == 'h' && frase[pos+1] == 'h' && frase[pos+2] == 'u') {
                out_evento->var = TIPO_ARRAY_U8; pos += 3; 
            }
            else if (frase[pos] == 'h' && frase[pos+1] == 'u') {
                out_evento->var = TIPO_ARRAY_U16; pos += 2; 
            }
            else if (frase[pos] == 'l' && frase[pos+1] == 'u') {
                out_evento->var = TIPO_ARRAY_U64; pos += 2; 
            }
            else if (frase[pos] == 'h' && frase[pos+1] == 'h' && frase[pos+2] == 'd') {
                out_evento->var = TIPO_ARRAY_I8; pos += 3; 
            }
            else if (frase[pos] == 'h' && frase[pos+1] == 'd') {
                out_evento->var = TIPO_ARRAY_I16; pos += 2; 
            }
            else if (frase[pos] == 'l' && frase[pos+1] == 'd') {
                out_evento->var = TIPO_ARRAY_I64; pos += 2; 
            }
            else if (frase[pos] == 'l' && frase[pos+1] == 'f') {
                out_evento->var = TIPO_ARRAY_F64; pos += 2; 
            }

            // Pula o ':' 
            if (frase[pos] == ':') {
                pos++;
            }
            // Trata o tamanho do array
            if (frase[pos] == '?') {
                out_evento->tamanho[0] = -1;
                pos++;
            } else {
                int tam = 0;
                while (frase[pos] >= '0' && frase[pos] <= '9') {
                    tam = tam * 10 + (frase[pos] - '0');
                    pos++;
                }
                out_evento->tamanho[0] = tam;
            }
            *inout_cursor = pos;
            return true;
        }

        // Variáveis normais (C99)
        if (frase[pos] == 'd') {
            out_evento->var = TIPO_I32; pos++; 
        } 
        else if (frase[pos] == 'f') {
            out_evento->var = TIPO_F32; pos++; 
        }
        else if (frase[pos] == 's') {
            out_evento->var = TIPO_STR; pos++; 
        }
        else if (frase[pos] == 'c') {
            out_evento->var = TIPO_C; pos++; 
        }
        else if (frase[pos] == 'u') {
            out_evento->var = TIPO_U32; pos++; 
        }
        else if (frase[pos] == 'h' && frase[pos+1] == 'h' && frase[pos+2] == 'u') {
            out_evento->var = TIPO_U8; pos += 3; 
        }
        else if (frase[pos] == 'h' && frase[pos+1] == 'u') {
            out_evento->var = TIPO_U16; pos += 2; 
        }
        else if (frase[pos] == 'l' && frase[pos+1] == 'u') {
            out_evento->var = TIPO_U64; pos += 2; 
        }
        else if (frase[pos] == 'h' && frase[pos+1] == 'h' && frase[pos+2] == 'd') {
            out_evento->var = TIPO_I8; pos += 3; 
        }
        else if (frase[pos] == 'h' && frase[pos+1] == 'd') {
            out_evento->var = TIPO_I16; pos += 2; 
        }
        else if (frase[pos] == 'l' && frase[pos+1] == 'd') {
            out_evento->var = TIPO_I64; pos += 2; 
        }
        else if (frase[pos] == 'l' && frase[pos+1] == 'f') {
            out_evento->var = TIPO_F64; pos += 2; 
        }

        *inout_cursor = pos;
        return true;
    }

    // Caso 4: (nome=)
    int i = 0;
    while (frase[pos] != '\0' && frase[pos] != ' ' && frase[pos] != '\t' && frase[pos] != '=') {
        if (i < 31) {
            out_evento->nome[i++] = frase[pos];
        }
        pos++;
    }
    out_evento->nome[i] = '\0';

    if (frase[pos] == '=') {
        out_evento->tipo = EVENTO_SEEK;
        pos++; 
        *inout_cursor = pos;
        return true;
    }

    // Se falhar tudo
    (*inout_cursor)++;
    return true;
}


static
int ufr_put_var(link_t* link, Evento* evento, va_list list) {
    int count = 0;

    switch (evento->var) {
        case TIPO_U8: {
            const uint8_t val = va_arg(list, int);
            // is_ok = link->enc_api->put_u8(link, &val, 1);
        }

        case TIPO_U16: {
            const uint16_t val = va_arg(list, int);
            // count = link->enc_api->put_u16(link, &val, 1);
        }

        case TIPO_U32: {
            const uint32_t val = va_arg(list, uint32_t);
            count = link->enc_api->put_u32(link, &val, 1);
        } break;

        case TIPO_U64: {
            const uint64_t val = va_arg(list, uint64_t);
            count = link->enc_api->put_u64(link, &val, 1);
        }

        case TIPO_I8: {
            const int8_t val = va_arg(list, int);
            // count = link->enc_api->put_i8(link, &val, 1);
        } break;

        case TIPO_I16: {
            const int16_t val = va_arg(list, int);
            // count = link->enc_api->put_i16(link, &val, 1);
        } break;

        case TIPO_I32: {
            const int32_t val = va_arg(list, int32_t);
            count = link->enc_api->put_i32(link, &val, 1);
        } break;

        case TIPO_I64: {
            const int64_t val = va_arg(list, int64_t);
            count = link->enc_api->put_i64(link, &val, 1);
        } break;

        case TIPO_F32: {
            const float val = (float) va_arg(list, double);
            count = link->enc_api->put_f32(link, &val, 1);
        } break;

        case TIPO_F64: {
            const double val = (double) va_arg(list, double);
            count = link->enc_api->put_f64(link, &val, 1);
        } break;

        case TIPO_STR: {
            const char* str = va_arg(list, const char*);
            count = link->enc_api->put_str(link, str);
        } break;

        case TIPO_ARRAY_U8: {
            const uint8_t* arr_ptr = va_arg(list, uint8_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            // ufr_put_au8(link, arr_ptr, arr_size);
        } break;

        case TIPO_ARRAY_U16: {
            const uint16_t* arr_ptr = va_arg(list, uint16_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            // ufr_put_au16(link, arr_ptr, arr_size);
        } break;

        case TIPO_ARRAY_U32: {
            const uint32_t* arr_ptr = va_arg(list, uint32_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            ufr_put_au32(link, arr_ptr, arr_size);
        } break;

        case TIPO_ARRAY_U64: {
            const uint64_t* arr_ptr = va_arg(list, uint64_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            // ufr_put_au64(link, arr_ptr, arr_size);
        } break;

        case TIPO_ARRAY_I8: {
            const int8_t* arr_ptr = va_arg(list, int8_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            // ufr_put_ai8(link, arr_ptr, arr_size);
        } break;

        case TIPO_ARRAY_I16: {
            const int16_t* arr_ptr = va_arg(list, int16_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            // ufr_put_ai16(link, arr_ptr, arr_size);
        } break;

        case TIPO_ARRAY_I32: {
            const int32_t* arr_ptr = va_arg(list, int32_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            count = ufr_put_ai32(link, arr_ptr, arr_size);
        }

        case TIPO_ARRAY_I64: {
            const int64_t* arr_ptr = va_arg(list, int64_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            // ufr_put_ai64(link, arr_ptr, arr_size);
        } break;

        case TIPO_ARRAY_F32: {
            const float* arr_ptr = va_arg(list, float*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            count = ufr_put_af32(link, arr_ptr, arr_size);
        } break;

        case TIPO_ARRAY_F64: {
            const double* arr_ptr = va_arg(list, double*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            count = ufr_put_af64(link, arr_ptr, arr_size);
        } break;

        default:
            break;
    }

    return count;
}


int ufr_put_va(link_t* link, const char* format, va_list list) {
    if ( link ) {
        if ( link->log_level > 0 ) {
            if ( link->enc_api == NULL ) {
                ufr_fatal(link, 0, "Encoder is not loaded");
            }
            if ( link->enc_api->cmd_seek_str == NULL ) {
                ufr_fatal(link, 0, "Function cmd_seek_str is NULL");
            }
            if ( link->enc_api->cmd_send == NULL ) {
                ufr_fatal(link, 0, "Function cmd_send is NULL");
            }
            if ( link->enc_api->put_u32 == NULL ) {
                ufr_fatal(link, -1, "Function put_u32 is NULL");
            }
            if ( link->enc_api->put_i32 == NULL ) {
                ufr_fatal(link, -1, "Function put_i32 is NULL");
            }
            if ( link->enc_api->put_f32 == NULL ) {
                ufr_fatal(link, -1, "Function put_f32 is NULL");
            }
            if ( link->state != UFR_STATE_PUT ) {
                // ufr_log_error(link, -1, "Link is not in PUT state");
            }
        }
    } else {
        ufr_fatal(link, -1, "Link is NULL");
    }

    int cursor = 0;
    int count = 0;
    Evento evento; 
    while (1) {
        if ( ufr_parse_frase(format, &cursor, &evento) == false ) {
            break;
        }

        if ( evento.tipo == EVENTO_VAR ) {
            count += ufr_put_var(link, &evento, list);

        } else if ( evento.tipo == EVENTO_SEEK ) {
            const int res = link->enc_api->cmd_seek_str(link, evento.nome);
            if ( res != UFR_OK ) {
                ufr_warn(link, "Field %s is not valid", evento.nome);
            }

        } else if ( evento.tipo == EVENTO_SEND ) {
            link->enc_api->cmd_send(link);
            link->put_count = 0;
            count += 1;
        } else {
            break;
        }

    }
    return count;
}


/* DEPRECIADA
int ufr_put_va(link_t* link, const char* format, va_list list) {
    if ( link ) {
        if ( link->log_level > 0 ) {
            if ( link->enc_api == NULL ) {
                ufr_fatal(link, 0, "Encoder is not loaded");
            }
            if ( link->enc_api->cmd_send == NULL ) {
                ufr_fatal(link, 0, "Function cmd_send is NULL");
            }
            if ( link->enc_api->put_u32 == NULL ) {
                ufr_fatal(link, -1, "Function put_u32 is NULL");
            }
            if ( link->enc_api->put_i32 == NULL ) {
                ufr_fatal(link, -1, "Function put_i32 is NULL");
            }
            if ( link->enc_api->put_f32 == NULL ) {
                ufr_fatal(link, -1, "Function put_f32 is NULL");
            }
            if ( link->state != UFR_STATE_PUT ) {
                // ufr_log_error(link, -1, "Link is not in PUT state");
            }
        }
    } else {
        ufr_fatal(link, -1, "Link is NULL");
    }

    int count = 0;
    char name[256];
    name[0] = '\0';
    uint8_t name_i = 0;
	while( true ) {
        const char type = *format;
        format += 1;

        // end of string
        if ( type == '\0' ) {
            break;
        }

        //
        if ( type == '#' ) {
            ufr_put_eof(link);

        //
        } else if ( type == '\n' ) {

            // case \n\n together
            if ( *format == '\n' ) {
                ufr_put_eof(link);
                format += 1;

            // case just one \n and no data in the link
            } else if ( link->put_count == 0 ) {
                ufr_put_eof(link);

            // case just one \n and there is data in the link to send
            } else {
                link->state = UFR_STATE_SEND;
			    link->enc_api->cmd_send(link);
                link->put_count = 0;
                ufr_put_begin_package(link);
            }

        //
        } else if ( type == '%' ) {
            const char type = *format;
            format += 1;

            // end of string
            if ( type == '\0' ) {
                break;
            }
       
		    if ( type == 'a' ) {
                const char arr_type = *format;
                format += 1;
                if ( arr_type == '\0' ) {
                    break;
                }
                const int32_t arr_size = va_arg(list, int32_t);
                if ( arr_type == 'i' ) {
                    const int32_t* arr_ptr = va_arg(list, int32_t*);
                    ufr_put_ai32(link, arr_ptr, arr_size);
                } else if ( arr_type == 'f' ) {
                    const float* arr_ptr = va_arg(list, float*);
                    ufr_put_af32(link, arr_ptr, arr_size);
                } else if ( arr_type == 'b' ) {
                    // const int8_t* arr_ptr = va_arg(list, int8_t*);
                    // ufr_put_raw(link, (uint8_t*) arr_ptr, arr_size);
                } 

            // s, i or f
            } else {
                switch (type) {
                    case 's': {
                        const char* str = va_arg(list, const char*);
                        link->enc_api->put_str(link, str);
                    } break;
                
                    case 'i':
                    case 'd': {
                        const int32_t val = va_arg(list, int32_t);
                        link->enc_api->put_i32(link, &val, 1);
                    } break;

                    case 'f': {
                        const float val = (float) va_arg(list, double);
                        link->enc_api->put_f32(link, &val, 1);
                    } break;

                    // pensar se adotar essa notacao
                    case '[': {
                        link->enc_api->cmd_enter(link, 100);
                    } break;

                    case ']': {
                        link->enc_api->cmd_leave(link);
                    } break;

                    default:
                        ufr_warn(link, "Operador '%c' nao definido", type);
                        break;
                }
                link->put_count += 1;
                count += 1;
            }

        // Case ':'
        } else if ( type == ':' ) {
            name[name_i] = '\0';
            if ( link->enc_api->cmd_seek_str != NULL ) {
                link->enc_api->cmd_seek_str(link, name);
            }

            name_i = 0;
            name[0] = '\0';

        // Case ' '
        } else if ( type == ' ' ) {
            // Descarta blank space

        // Case of A-Za-z
        } else {
            name[name_i] = type;
            name_i += 1;
        }
    }
    return count;
}
*/


int ufr_put(link_t* link, const char* format, ...) {
    va_list list;
    va_start(list, format);
    const int nitems = ufr_put_va(link, format, list);
    va_end(list);
    return nitems;
}

int ufr_putln(link_t* link, const char* format, ...) {
    va_list list;
    va_start(list, format);
    const int nitems = ufr_put_va(link, format, list);
    va_end(list);
    ufr_send(link);
    return nitems;
}

int ufr_put_u32(link_t* link, const uint32_t val) {
    return link->enc_api->put_u32(link, &val, 1);

}

int ufr_put_i32(link_t* link, const int32_t val) {
    return link->enc_api->put_i32(link, &val, 1);

}

int ufr_put_f32(link_t* link, const float val) {
    // check inputs
    if ( link != NULL && link->log_level > 0 ) {
        if ( link->enc_api == NULL ) {
            ufr_fatal(link, 1, "Encoder is NULL");
        }
        if ( link->enc_api->put_f32 == NULL ) {
            ufr_fatal(link, 1, "enc_api->put_f32 is NULL");
        }
    }
    // send data
    return link->enc_api->put_f32(link, &val, 1);
}



int ufr_put_pu32(link_t* link, const uint32_t* array, int nitems) {
    const int wrote_nitems = link->enc_api->put_u32(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}

int ufr_put_pi32(link_t* link, const int32_t* array, int nitems) {
    const int wrote_nitems = link->enc_api->put_i32(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}

int ufr_put_pf32(link_t* link, const float* array, int nitems) {
    // check inputs
    if ( link != NULL && link->log_level > 0 ) {
        if ( link->enc_api == NULL ) {
            ufr_fatal(link, 1, "Encoder is NULL");
        }
        if ( link->enc_api->put_f32 == NULL ) {
            ufr_fatal(link, 1, "enc_api->put_f32 is NULL");
        }
    }
    // send data
    const int wrote_nitems = link->enc_api->put_f32(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}

// 64bits

int ufr_put_pu64(link_t* link, const uint64_t* array, int nitems) {
    const int wrote_nitems = link->enc_api->put_u64(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}

int ufr_put_pi64(link_t* link, const int64_t* array, int nitems) {
    const int wrote_nitems = link->enc_api->put_i64(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}

int ufr_put_pf64(link_t* link, const double* array, int nitems) {
    const int wrote_nitems = link->enc_api->put_f64(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}




int ufr_put_au32(link_t* link, const uint32_t array[], int nitems) {
    if ( link->log_level > 0 ) {
        if ( link->enc_api == NULL ) {
            return ufr_error(link, 0, "Encoder is null");
        }
        if ( link->enc_api->cmd_enter == NULL ) {
            return ufr_error(link, 0, "Function enter of encoder is null");
        }
        if ( link->enc_api->cmd_leave == NULL ) {
            return ufr_error(link, 0, "Function leave of encoder is null");
        }
        if ( link->enc_api->put_u32 == NULL ) {
            return ufr_error(link, 0, "Function put_u32 of encoder is null");
        }
    }

    if ( link->enc_api->cmd_enter(link, nitems) != UFR_OK ) {
        return -1;
    }

    const int wrote_nitems = link->enc_api->put_u32(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    if ( link->enc_api->cmd_leave(link) != UFR_OK ) {
        ufr_warn(link, "Function leave returned with error");
    }
    return wrote_nitems;
}

int ufr_put_ai32(link_t* link, const int32_t array[], int nitems) {
    if ( link->log_level > 0 ) {
        if ( link->enc_api == NULL ) {
            return ufr_error(link, 0, "Encoder is null");
        }
        if ( link->enc_api->cmd_enter == NULL ) {
            return ufr_error(link, 0, "Function enter of encoder is null");
        }
        if ( link->enc_api->cmd_leave == NULL ) {
            return ufr_error(link, 0, "Function leave of encoder is null");
        }
        if ( link->enc_api->put_i32 == NULL ) {
            return ufr_error(link, 0, "Function put_i32 of encoder is null");
        }
    }

    if ( link->enc_api->cmd_enter(link, nitems) != UFR_OK ) {
        return -1;
    }

    const int wrote_nitems = link->enc_api->put_i32(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    if ( link->enc_api->cmd_leave(link) != UFR_OK ) {
        ufr_warn(link, "Function leave returned with error");
    }
    return wrote_nitems;
}

int ufr_put_af32(link_t* link, const float array[], int nitems) {
    if ( link->log_level > 0 ) {
        if ( link->enc_api == NULL ) {
            return ufr_error(link, 0, "Encoder is null");
        }
        if ( link->enc_api->cmd_enter == NULL ) {
            return ufr_error(link, 0, "Function enter of encoder is null");
        }
        if ( link->enc_api->cmd_leave == NULL ) {
            return ufr_error(link, 0, "Function leave of encoder is null");
        }
        if ( link->enc_api->put_f32 == NULL ) {
            return ufr_error(link, 0, "Function put_f32 of encoder is null");
        }
    }

    if ( link->enc_api->cmd_enter(link, nitems) != UFR_OK ) {
        return -1;
    }

    const int wrote_nitems = link->enc_api->put_f32(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    if ( link->enc_api->cmd_leave(link) != UFR_OK ) {
        ufr_warn(link, "Function leave returned with error");
    }
    return wrote_nitems;
}

int ufr_put_af64(link_t* link, const double array[], int nitems) {
    if ( link->log_level > 0 ) {
        if ( link->enc_api == NULL ) {
            return ufr_error(link, 0, "Encoder is null");
        }
        if ( link->enc_api->cmd_enter == NULL ) {
            return ufr_error(link, 0, "Function enter of encoder is null");
        }
        if ( link->enc_api->cmd_leave == NULL ) {
            return ufr_error(link, 0, "Function leave of encoder is null");
        }
        if ( link->enc_api->put_f64 == NULL ) {
            return ufr_error(link, 0, "Function put_f64 of encoder is null");
        }
    }

    if ( link->enc_api->cmd_enter(link, nitems) != UFR_OK ) {
        return -1;
    }

    const int wrote_nitems = link->enc_api->put_f64(link, array, nitems);
    if ( wrote_nitems > 0 ) {
        link->put_count += wrote_nitems;
    }
    if ( link->enc_api->cmd_leave(link) != UFR_OK ) {
        ufr_warn(link, "Function leave returned with error");
    }
    return wrote_nitems;
}





















int ufr_put_eof(link_t* link) {
    if ( link->type_started == UFR_START_SUBSCRIBER ) {
        return ufr_error(link, -1, "Link is a subscriber");
    }

    if ( link->enc_api == NULL ) {
        return ufr_error(link, 1, "link->enc_api == NULL");
    }

    if ( link->enc_api->cmd_eof == NULL ) {
        //ufr_fatal(link, "link->enc_api->cmd_eof == NULL");
        return ufr_error(link, 1, "link->enc_api->cmd_eof == NULL");
    }

    // send the last message
    link->state = UFR_STATE_SEND_LAST;
    const int retval = link->enc_api->cmd_eof(link);

    // update the state of the link
    if ( retval == UFR_OK ) {
        ufr_set_state_ready(link);
    }
    return retval;
}

int ufr_put_str(link_t* link, const char* value) {
    return link->enc_api->put_str(link, value);
}


int ufr_put_raw(link_t* link, const uint8_t* buffer, int nitems) {
    // check inputs
    if ( link != NULL && link->enc_api->put_raw == NULL ) {
        ufr_fatal(link, 1, "enc_api->put_raw is NULL");
    }
    // send data
    const int wrote_nitems = link->enc_api->put_raw(link, buffer, nitems);
    if ( wrote_nitems > 0 ) {
        ufr_log(link, "wrote %ld bytes", nitems);
        link->put_count += wrote_nitems;
    }
    return wrote_nitems;
}


int ufr_put_bin(link_t* link, const char* mime, const char* buffer, int nbytes) {
    // check inputs
    if ( link != NULL && link->enc_api->put_bin == NULL ) {
        ufr_fatal(link, 1, "enc_api->put_bin is NULL");
    }
    // send data
    const int wrote_nbytes = link->enc_api->put_bin(link, mime, buffer, nbytes);
    if ( wrote_nbytes > 0 ) {
        link->put_count += wrote_nbytes;
    }
    return wrote_nbytes;
}

int ufr_put_file(link_t* link, const char* mime, const char* buffer, int nbytes) {
    return ufr_put_bin(link, mime, buffer, nbytes);
}



int ufr_put_enter(link_t* link, int max_nitems) {
    if (link->enc_api->cmd_enter == NULL ) {
        return ufr_error(link, 1, "enter array pointer is NULL");
    }

    return link->enc_api->cmd_enter(link, max_nitems);
}

int ufr_put_leave(link_t* link) {
    if (link->enc_api->cmd_leave == NULL ) {
        return ufr_error(link, 1, "leave array pointer is NULL");
    }

    return link->enc_api->cmd_leave(link);
}


int ufr_put_begin_package(link_t* link) {
    // call the ready state
    // link->state = UFR_STATE_READY;
    link->put_count = 0;
    if ( link->gtw_api->ready != NULL ) {
        link->gtw_api->ready(link);
    }

    // set next state
    /*if ( link->type_started == UFR_START_PUBLISHER || link->type_started == UFR_START_CLIENT ) {
        link->state = UFR_STATE_PUT;
    } else if ( link->type_started == UFR_START_SUBSCRIBER || link->type_started == UFR_START_SERVER ) {
        link->state = UFR_STATE_RECV;
    }*/

    // sucess
    return UFR_OK;
}