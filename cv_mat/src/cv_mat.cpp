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

#include "cv_mat.h"

#include <new>
#include <opencv2/opencv.hpp>

using namespace cv;

// ============================================================================
//  CV MAT
// ============================================================================

extern "C"
cv_mat_t cv_mat_new(int rows, int cols, int type) {
    cv_mat_t mat;
    cv_mat_init(&mat, rows, cols, type);
    return mat;
}

extern "C"
void cv_mat_init(cv_mat_t* mat_c, int rows, int cols, int type) {
    new (mat_c) Mat(rows,cols,type);
}

extern "C"
void cv_mat_init_with_ptr(cv_mat_t* mat_c, int rows, int cols, int type, void* data) {
    new (mat_c) Mat(rows,cols,type,data);
}

extern "C"
void cv_mat_free(cv_mat_t* mat_c) {
    Mat* mat_cpp = (Mat*) mat_c;
    mat_cpp->~Mat();
}

extern "C"
int  cv_mat_rows(const cv_mat_t* mat_c) {
    Mat* mat_cpp = (Mat*) mat_c;
    return mat_cpp->rows;
}

extern "C"
int  cv_mat_cols(const cv_mat_t* mat_c) {
    Mat* mat_cpp = (Mat*) mat_c;
    return mat_cpp->cols;
}

extern "C"
uint8_t* cv_mat_data_u8(const cv_mat_t* mat_c) {
    Mat* mat_cpp = (Mat*) mat_c;
    return (uint8_t*) mat_cpp->data;
}

// ============================================================================
//  CV
// ============================================================================

extern "C"
cv_mat_t cv_imread(const char* filename) {
    cv_mat_t mat;
    cv_mat_init(&mat, 0, 0, CV_8UC1);
    Mat* mat_cpp = (Mat*) &mat;
    *mat_cpp = imread(filename);
    return mat;
}

extern "C"
void cv_imwrite(const char* filename, const cv_mat_t* src) {
    Mat* mat_cpp = (Mat*) src;
    imwrite(std::string(filename), *mat_cpp);
}

extern "C"
void cv_imshow(const char* name, cv_mat_t* mat_c) {
    Mat* mat_cpp = (Mat*) mat_c;
    imshow(std::string(name), *mat_cpp);
}

extern "C"
int cv_waitkey(int time) {
    return waitKey(time);
}

extern "C"
void cv_cvtColor(cv_mat_t* src, cv_mat_t* dst, int type) {
    Mat* src_cpp = (cv::Mat*) src;
    Mat* dst_cpp = (cv::Mat*) dst;
    cvtColor(*src_cpp, *dst_cpp, type);
}






