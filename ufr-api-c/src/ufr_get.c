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

#include <stdlib.h>
#include <string.h>

#include "ufr.h"

// ============================================================================
//  Private Functions
// ============================================================================

int ufr_get_scalar(link_t* link, const Evento* evento, va_list list) {
    int count = 0;

    switch (evento->var) {
        case TIPO_U8: {
            uint8_t* val = va_arg(list, uint8_t*);
            count = link->dcr_api->get_u8(link, val, 1);
        }

        case TIPO_U16: {
            uint16_t* val = va_arg(list, uint16_t*);
            count = link->dcr_api->get_u16(link, val, 1);
        }

        case TIPO_U32: {
            uint32_t* val = va_arg(list, uint32_t*);
            count = link->dcr_api->get_u32(link, val, 1);
        } break;

        case TIPO_U64: {
            uint64_t* val = va_arg(list, uint64_t*);
            count = link->dcr_api->get_u64(link, val, 1);
        }

        case TIPO_I8: {
            int8_t* val = va_arg(list, int8_t*);
            count = link->dcr_api->get_i8(link, val, 1);
        } break;

        case TIPO_I16: {
            int16_t* val = va_arg(list, int16_t*);
            count = link->dcr_api->get_i16(link, val, 1);
        } break;

        case TIPO_I32: {
            int32_t* val = va_arg(list, int32_t*);
            count = link->dcr_api->get_i32(link, val, 1);
        } break;

        case TIPO_I64: {
            int64_t* val = va_arg(list, int64_t*);
            count = link->dcr_api->get_i64(link, val, 1);
        } break;

        case TIPO_F32: {
            float* val = va_arg(list, float*);
            count = link->dcr_api->get_f32(link, val, 1);
        } break;

        case TIPO_F64: {
            double* val = va_arg(list, double*);
            count = link->dcr_api->get_f64(link, val, 1);
        } break;

        case TIPO_STR: {
            char* str = va_arg(list, char*);
            link->dcr_api->get_str(link, str, 1024);
            count += 1;
        } break;

        default:
            break;
    }

    return count;
}

int ufr_get_array(link_t* link, const Evento* evento, va_list list) {
    int count = 0;

    switch (evento->var) {
        case TIPO_U8: {
            uint8_t* arr_ptr = va_arg(list, uint8_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            ufr_get_arr_u8(link, arr_ptr, arr_size);
        }

        case TIPO_U16: {
            uint16_t* arr_ptr = va_arg(list, uint16_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            ufr_get_arr_u16(link, arr_ptr, arr_size);
        }

        case TIPO_U32: {
            uint32_t* arr_ptr = va_arg(list, uint32_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            ufr_get_arr_u32(link, arr_ptr, arr_size);
        } break;

        case TIPO_U64: {
            uint64_t* arr_ptr = va_arg(list, uint64_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            ufr_get_arr_u64(link, arr_ptr, arr_size);
        }

        case TIPO_I8: {
            int8_t* arr_ptr = va_arg(list, int8_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            ufr_get_arr_i8(link, arr_ptr, arr_size);
        } break;

        case TIPO_I16: {
            int16_t* arr_ptr = va_arg(list, int16_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            ufr_get_arr_i16(link, arr_ptr, arr_size);
        } break;

        case TIPO_I32: {
            int32_t* arr_ptr = va_arg(list, int32_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            const int res = ufr_get_arr_i32(link, arr_ptr, arr_size);
            if ( res > 0 ) {
                count = 1;
            }
        } break;

        case TIPO_I64: {
            int64_t* arr_ptr = va_arg(list, int64_t*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            ufr_get_arr_i64(link, arr_ptr, arr_size);
        } break;

        case TIPO_F32: {
            float* arr_ptr = va_arg(list, float*);
            // const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            const int count_array = ufr_get_arr_f32(link, arr_ptr, 1000);
            if (evento->tamanho[0] == -1) {
                int32_t* out_tam = va_arg(list, int32_t);
                *out_tam = count_array;
            }
            count += 1;
        } break;

        case TIPO_F64: {
            double* arr_ptr = va_arg(list, double*);
            const int32_t arr_size = (evento->tamanho[0] == -1) ? va_arg(list, int32_t) : evento->tamanho[0];
            count = ufr_get_arr_f64(link, arr_ptr, arr_size);
        } break;

        case TIPO_STR: {
            
        } break;

        default:
            break;
    }

    return count;
}

// ============================================================================
//  Get
// ============================================================================

int ufr_get_va(link_t* link, const char* format, va_list list) {
    if ( link ) {
        if ( link->log_level > 0 ) {
            if ( link->dcr_api == NULL ) {
                ufr_fatal(link, 0, "Decoder is not loaded");
            }
            if ( link->dcr_api->get_u32 == NULL ) {
                ufr_fatal(link, 0, "Function get_u32 is NULL");
            }
            if ( link->dcr_api->get_i32 == NULL ) {
                ufr_fatal(link, -1, "Function get_i32 is NULL");
            }
            if ( link->dcr_api->get_f32 == NULL ) {
                ufr_fatal(link, -1, "Function get_f32 is NULL");
            }
            if ( link->dcr_api->get_str == NULL ) {
                ufr_fatal(link, -1, "Function get_str is NULL");
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

        if ( evento.tipo == EVENTO_VAR_SCALAR ) {
            count += ufr_get_scalar(link, &evento, list);

        } else if ( evento.tipo == EVENTO_VAR_ARRAY ) {
            count += ufr_get_array(link, &evento, list);

        } else if ( evento.tipo == EVENTO_SEEK ) {
            /*const int res = link->dcr_api->cmd_seek_str(link, evento.nome);
            if ( res != UFR_OK ) {
                ufr_warn(link, "Field %s is not valid", evento.nome);
            }*/

        } else if ( evento.tipo == EVENTO_RECV1 ) {
            if ( ufr_recv(link) == false ) {
                return ufr_error(link, -1, "Error to receive data");
            }
            count += 1;

        } else {
            break;
        }

    }
    return count;
}


int ufr_get(link_t* link, const char* format, ...) {
    va_list list;
    va_start(list, format);
    const int retval = ufr_get_va(link, format, list);
    va_end(list);
    return retval;
}


void ufr_get_eof(link_t* link) {
    uint8_t count;
    for (count=0; count<32; count++) {
        if ( ufr_recv(link) == false ) {
            break;
        }
    }
    if ( count == 32 ) {
        ufr_fatal(link, -1, "Error to get EOF");
    }
    ufr_set_state_ready(link);
}


const uint8_t* ufr_get_rawptr(link_t* link) {
    return link->dcr_api->get_ptr(link);
    // return NULL;
    // return link->dcr_api->get_rawptr(link);
}



// GET 32 bites

uint32_t ufr_get_u32(link_t* link, uint32_t defval) {
    uint32_t retval;
    if ( link->dcr_api->get_u32(link, &retval, 1) != 1 ) {
        retval = defval;
    }
    return retval;
}

int32_t ufr_get_i32(link_t* link, int32_t defval) {
    int32_t retval;
    if ( link->dcr_api->get_i32(link, &retval, 1) != 1 ) {
        retval = defval;
    }
    return retval;
}

float ufr_get_f32(link_t* link, float defval) {
    float retval;
    if ( link->dcr_api->get_f32(link, &retval, 1) != 1 ) {
        retval = defval;
    }
    return retval;
}

int ufr_get_pf32(link_t* link, float buffer[], int max_nitems) {
    return link->dcr_api->get_f32(link, buffer, max_nitems);
}

// GET 64 bites

uint64_t ufr_get_u64(link_t* link, uint64_t defval) {
    uint64_t retval;
    if ( link->dcr_api->get_u64(link, &retval, 1) != 1 ) {
        retval = defval;
    }
    return retval;
}

int64_t ufr_get_i64(link_t* link, int64_t defval) {
    int64_t retval;
    if ( link->dcr_api->get_i64(link, &retval, 1) != 1 ) {
        retval = defval;
    }
    return retval;
}

double ufr_get_f64(link_t* link, double defval) {
    double retval;
    if ( link->dcr_api->get_f64(link, &retval, 1) != 1 ) {
        retval = defval;
    }
    return retval;
}

// Arrays

int ufr_get_str(link_t* link, char* buffer, int maxlen) {
    buffer[0] = '\0';
    const int is_ok = link->dcr_api->get_str(link, buffer, maxlen);
    return is_ok == UFR_OK;
}

int ufr_get_raw(link_t* link, uint8_t* buffer, int max_nitems) {
    size_t arr_size = 0;
    link->dcr_api->get_raw(link, buffer, max_nitems);
    return arr_size;
}

int ufr_get_bin(link_t* link, char** out_mime, char** out_data, int* out_nbytes) {

    if (link->log_level > 0 ) {
        if ( link != NULL && link->dcr_api->get_bin == NULL ) {
            ufr_fatal(link, 1, "dcr_api->get_bin is NULL");
        }
        if ( out_mime == NULL ) {
            ufr_fatal(link, 1, "out_mime is NULL");
        }
        if ( out_data == NULL ) {
            ufr_fatal(link, 1, "out_data is NULL");
        }
        if ( out_nbytes == NULL ) {
            ufr_fatal(link, 1, "out_nbytes is NULL");
        }
    }

    // Set zero to output
    *out_mime = NULL;
    *out_data = NULL;
    *out_nbytes = 0;

    // execute decoder get_bin
    return link->dcr_api->get_bin(link, out_mime, out_data, out_nbytes);
}


// Enter and Leave

int ufr_get_enter(link_t* link) {
    if (link->dcr_api->cmd_enter == NULL ) {
        return ufr_error(link, 1, "Function enter in Decoder is NULL");
    }
    return link->dcr_api->cmd_enter(link);
}

int ufr_get_leave(link_t* link) {
    if (link->dcr_api->cmd_leave == NULL ) {
        return ufr_error(link, 1, "Function leave in Decoder is NULL");
    }
    return link->dcr_api->cmd_leave(link);
}


int ufr_get_arr_u8(link_t* link, uint8_t buffer[], int max_items) {
    const int res1 = link->dcr_api->cmd_enter(link);
    if ( res1 != UFR_OK ) {
        return ufr_error(link, -1, "Error on Enter function");
    }
    const int items_read = link->dcr_api->get_u8(link, buffer, max_items);
    if ( items_read < 0 ) {
        return ufr_error(link, -1, "Error on get_u8 function");
    }
    link->dcr_api->cmd_leave(link);
    return items_read;
}

int ufr_get_arr_i8(link_t* link, int8_t buffer[], int max_items) {
    const int res1 = link->dcr_api->cmd_enter(link);
    if ( res1 != UFR_OK ) {
        return ufr_error(link, -1, "Error on Enter function");
    }
    const int items_read = link->dcr_api->get_i8(link, buffer, max_items);
    if ( items_read < 0 ) {
        return ufr_error(link, -1, "Error on get_i8 function");
    }
    link->dcr_api->cmd_leave(link);
    return items_read;
}

int ufr_get_arr_u16(link_t* link, uint16_t buffer[], int max_items) {
    const int res1 = link->dcr_api->cmd_enter(link);
    if ( res1 != UFR_OK ) {
        return ufr_error(link, -1, "Error on Enter function");
    }
    const int items_read = link->dcr_api->get_u16(link, buffer, max_items);
    if ( items_read < 0 ) {
        return ufr_error(link, -1, "Error on get_u16 function");
    }
    link->dcr_api->cmd_leave(link);
    return items_read;
}

int ufr_get_arr_i16(link_t* link, int16_t buffer[], int max_items) {
    const int res1 = link->dcr_api->cmd_enter(link);
    if ( res1 != UFR_OK ) {
        return ufr_error(link, -1, "Error on Enter function");
    }
    const int items_read = link->dcr_api->get_i16(link, buffer, max_items);
    if ( items_read < 0 ) {
        return ufr_error(link, -1, "Error on get_i16 function");
    }
    link->dcr_api->cmd_leave(link);
    return items_read;
}

int ufr_get_arr_u32(link_t* link, uint32_t buffer[], int max_items) {
    const int res1 = link->dcr_api->cmd_enter(link);
    if ( res1 != UFR_OK ) {
        return ufr_error(link, -1, "Error on Enter function");
    }
    const int items_read = link->dcr_api->get_u32(link, buffer, max_items);
    if ( items_read < 0 ) {
        return ufr_error(link, -1, "Error on get_u32 function");
    }
    link->dcr_api->cmd_leave(link);
    return items_read;
}

int ufr_get_arr_i32(link_t* link, int32_t buffer[], int max_items) {
    const int res1 = link->dcr_api->cmd_enter(link);
    if ( res1 != UFR_OK ) {
        return ufr_error(link, -1, "Error on Enter function");
    }
    const int items_read = link->dcr_api->get_i32(link, buffer, max_items);
    if ( items_read < 0 ) {
        return ufr_error(link, -1, "Error on get_i32 function");
    }
    link->dcr_api->cmd_leave(link);
    return items_read;
}

int ufr_get_arr_u64(link_t* link, uint64_t buffer[], int max_items) {
    const int res1 = link->dcr_api->cmd_enter(link);
    if ( res1 != UFR_OK ) {
        return ufr_error(link, -1, "Error on Enter function");
    }
    const int items_read = link->dcr_api->get_u64(link, buffer, max_items);
    if ( items_read < 0 ) {
        return ufr_error(link, -1, "Error on get_u64 function");
    }
    link->dcr_api->cmd_leave(link);
    return items_read;
}

int ufr_get_arr_i64(link_t* link, int64_t buffer[], int max_items) {
    const int res1 = link->dcr_api->cmd_enter(link);
    if ( res1 != UFR_OK ) {
        return ufr_error(link, -1, "Error on Enter function");
    }
    const int items_read = link->dcr_api->get_i64(link, buffer, max_items);
    if ( items_read < 0 ) {
        return ufr_error(link, -1, "Error on get_i64 function");
    }
    link->dcr_api->cmd_leave(link);
    return items_read;
}


int ufr_get_arr_f32(link_t* link, float buffer[], int max_items) {
    if (link->dcr_api->cmd_leave == NULL ) {
        return ufr_error(link, 1, "Function leave in Decoder is NULL");
    }

    const int res1 = link->dcr_api->cmd_enter(link);
    if ( res1 != UFR_OK ) {
        return ufr_error(link, -1, "Error on Enter function");
    }
    const int items_read = link->dcr_api->get_f32(link, buffer, max_items);
    if ( items_read < 0 ) {
        return ufr_error(link, -1, "Error on get_f32 function");
    }
    link->dcr_api->cmd_leave(link);
    return items_read;
}

int ufr_get_arr_f64(link_t* link, double buffer[], int max_items) {
    if (link->dcr_api->cmd_leave == NULL ) {
        return ufr_error(link, 1, "Function leave in Decoder is NULL");
    }

    const int res1 = link->dcr_api->cmd_enter(link);
    if ( res1 != UFR_OK ) {
        return ufr_error(link, -1, "Error on Enter function");
    }
    const int items_read = link->dcr_api->get_f64(link, buffer, max_items);
    if ( items_read < 0 ) {
        return ufr_error(link, -1, "Error on get_f64 function");
    }
    link->dcr_api->cmd_leave(link);
    return items_read;
}




// ============================================================================
//  POSIX
// ============================================================================

#define UFR_UFILE_STDOUT   0
#define UFR_UFILE_STDIN    1
#define UFR_UFILE_STDERR   2


UFILE* g_files[4] = {NULL, NULL, NULL, NULL};

int ufr_fprintf(UFILE* fd, const char* format, ...) {
    va_list list;
    va_start(list, format);
    const int nitems = ufr_put_va(fd, format, list);
    va_end(list);
    return nitems;
}

int ufr_printf(const char* format, ...) {
    if ( g_files[UFR_UFILE_STDOUT] == NULL ) {
        ufr_stdout("@new posix:stdout @coder text");
    }

    va_list list;
    va_start(list, format);
    const int nitems = ufr_put_va(g_files[UFR_UFILE_STDOUT], format, list);
    va_end(list);
    return nitems;
}


int ufr_fscanf(UFILE* fd, const char* format, ...) {
    va_list list;
    va_start(list, format);
    const int nitems = ufr_get_va(fd, format, list);
    va_end(list);
    return nitems;
}

int ufr_scanf(const char* format, ...) {
    if ( g_files[UFR_UFILE_STDIN] == NULL ) {
        ufr_stdin("@new posix:stdin @coder text");
    }

    va_list list;
    va_start(list, format);
    const int nitems = ufr_get_va(g_files[UFR_UFILE_STDIN], format, list);
    va_end(list);
    return nitems;
}



static
int ufr_stdin_args(const char* format, const ufr_args_t* args) {
    if ( g_files[UFR_UFILE_STDIN] != NULL ) {
        ufr_close(g_files[UFR_UFILE_STDIN]);
        g_files[UFR_UFILE_STDIN] = NULL;
    }

    // Open link
    link_t* link = malloc(sizeof(link_t));
    if ( link == NULL ) {
        return -1;
    }
    ufr_subscriber_args(link, args);

    // success
    g_files[UFR_UFILE_STDIN] = link;
    return UFR_OK;
}

int ufr_stdin(const char* format, ...) {
    // load variable arguments to args
    ufr_args_t args;
    va_list list;
    va_start(list, format);
    ufr_args_load_from_va(&args, format, list);
    va_end(list);

    // 
    return ufr_stdin_args(format, &args);
}

int ufr_stdin_env(const char* name) {
    const char* text = getenv(name);
    if ( text == NULL ){
        ufr_fatal(NULL, 1, "Environment variable %s is not defined");
    }

    //
    const ufr_args_t args = {.text=text};
    return ufr_stdin_args(name, &args);
}



static
int ufr_stdout_args(const char* format, const ufr_args_t* args) {
    if ( g_files[UFR_UFILE_STDOUT] != NULL ) {
        ufr_close(g_files[UFR_UFILE_STDOUT]);
        g_files[UFR_UFILE_STDOUT] = NULL;
    }

    // Open link
    link_t* link = malloc(sizeof(link_t));
    if ( link == NULL ) {
        return -1;
    }
    ufr_publisher_args(link, args);

    // success
    g_files[UFR_UFILE_STDOUT] = link;
    return UFR_OK;
}

int ufr_stdout(const char* format, ...) {
    // load variable arguments to args
    ufr_args_t args;
    va_list list;
    va_start(list, format);
    ufr_args_load_from_va(&args, format, list);
    va_end(list);

    // 
    return ufr_stdout_args(format, &args);
}

int ufr_stdout_env(const char* name) {
    const char* text = getenv(name);
    if ( text == NULL ){
        ufr_fatal(NULL, 1, "Environment variable %s is not defined");
    }

    //
    const ufr_args_t args = {.text=text};
    return ufr_stdout_args(name, &args);
}