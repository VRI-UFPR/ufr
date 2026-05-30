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

#include "ufr.h"

// ============================================================================
//  Meta
// ============================================================================

const char* ufr_meta_item_mime(const link_t* link) {
    return link->dcr_api->meta_item_mime(link);
}

char ufr_meta_item_type(const link_t* link) {
    return link->dcr_api->meta_item_type(link);
}

int ufr_meta_item_nbytes(const link_t* link) {
    return link->dcr_api->meta_item_nbytes(link);
}

int ufr_meta_item_nitems(const link_t* link) {
    return link->dcr_api->meta_item_nitems(link);
}



const char* ufr_meta_pack_mime(const link_t* link) {
    if ( link->dcr_api->meta_pack_mime == NULL ) {
        ufr_error(link, -1, "Function meta_pack_mime from decoder is NULL");
        return "";
    }
    return link->dcr_api->meta_pack_mime(link);
}

int ufr_meta_pack_nbytes(const link_t* link) {
    return link->dcr_api->meta_pack_nbytes(link);
}

int ufr_meta_pack_nitems(const link_t* link) {
    return link->dcr_api->meta_pack_nitems(link);
}

