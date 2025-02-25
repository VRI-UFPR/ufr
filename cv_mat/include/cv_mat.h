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

#pragma once

#include <stdint.h>


#define CV_CN_MAX   512
#define CV_CN_SHIFT   3
#define CV_DEPTH_MAX   (1 << CV_CN_SHIFT)

#define CV_8U   0
#define CV_8S   1
#define CV_16U   2
#define CV_16S   3
#define CV_32S   4
#define CV_32F   5
#define CV_64F   6
#define CV_USRTYPE1   7
#define CV_MAT_DEPTH_MASK   (CV_DEPTH_MAX - 1)
#define CV_MAT_DEPTH(flags)   ((flags) & CV_MAT_DEPTH_MASK)
#define CV_MAKETYPE(depth, cn)   (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))
#define CV_MAKE_TYPE   CV_MAKETYPE

#define CV_8UC1     CV_MAKETYPE(CV_8U,1)
#define CV_8UC2     CV_MAKETYPE(CV_8U,2)
#define CV_8UC3     CV_MAKETYPE(CV_8U,3)
#define CV_8UC4     CV_MAKETYPE(CV_8U,4)
#define CV_8UC(n)   CV_MAKETYPE(CV_8U,(n))


typedef struct {
    uint32_t data[32];
} cv_mat_t;

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
//  CV MAT
// ============================================================================

cv_mat_t cv_mat_new(int rows, int cols, int type);
void cv_mat_init(cv_mat_t* mat, int rows, int cols, int type);
void cv_mat_init_with_ptr(cv_mat_t* mat_c, int rows, int cols, int type, void* data);
void cv_mat_free(cv_mat_t* mat);
int  cv_mat_rows(const cv_mat_t* mat);
int  cv_mat_cols(const cv_mat_t* mat);
uint8_t* cv_mat_data_u8(const cv_mat_t* mat);

// ============================================================================
//  CV
// ============================================================================

cv_mat_t cv_imread(const char* filename);
void cv_imwrite(const char* filename, const cv_mat_t* src);
void cv_imshow(const char* name, cv_mat_t* mat);
int  cv_waitkey(int time);
void cv_cvtColor(cv_mat_t* src, cv_mat_t* dst, int type);

// ============================================================================
//  Footer
// ============================================================================

#ifdef __cplusplus
}
#endif







