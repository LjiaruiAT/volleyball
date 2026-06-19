#include "arm_math.h"
#include <math.h>
#include <string.h>
#include "matrix.h"

#define MAT(A, r, c) ((A)->pData[(r)*(A)->numCols + (c)])

/* 交换两个 int */
static inline void swap_int(int *a, int *b)
{
    int t = *a; *a = *b; *b = t;
}

/* 交换矩阵的两列 */
static void swap_cols_f32(arm_matrix_instance_f32 *A, int c1, int c2)
{
    uint32_t M = A->numRows;
    for (uint32_t i = 0; i < M; i++) {
        float tmp = MAT(A, i, c1);
        MAT(A, i, c1) = MAT(A, i, c2);
        MAT(A, i, c2) = tmp;
    }
}

/* 计算列范数（从 start_row 开始） */
static float col_norm_f32(
    const arm_matrix_instance_f32 *A,
    int col,
    int start_row)
{
    float sum = 0.0f;
    for (uint32_t i = start_row; i < A->numRows; i++) {
        float v = MAT(A, i, col);
        sum += v * v;
    }
    return sqrtf(sum);
}

/* 计算 Householder 向量，返回 beta */
static float compute_householder(float *v, int len)
{
    float sigma = 0.0f;
    for (int i = 1; i < len; i++)
        sigma += v[i] * v[i];

    if (sigma == 0.0f)
        return 0.0f;

    float mu = sqrtf(v[0]*v[0] + sigma);
    if (v[0] <= 0)
        v[0] -= mu;
    else
        v[0] = -sigma / (v[0] + mu);

    float beta = 2.0f * v[0] * v[0] / (sigma + v[0]*v[0]);

    for (int i = 1; i < len; i++)
        v[i] /= v[0];

    v[0] = 1.0f;
    return beta;
}

/* ======================= 主函数 ======================= */

int solve_linear_system_qr_f32(
    const arm_matrix_instance_f32 *A_in,
    const arm_matrix_instance_f32 *b_in,
    arm_matrix_instance_f32 *x_out)
{
    uint32_t M = A_in->numRows;
    uint32_t N = A_in->numCols;

    if (b_in->numRows != M || b_in->numCols != 1 ||
        x_out->numRows != N || x_out->numCols != 1)
        return -1;

    /* 工作区（可改成外部传入以避免栈占用） */
    float A_buf[24];
    float b_buf[8];
    float v[8];
    float col_norms[3];
    int   col_perm[3];

    arm_matrix_instance_f32 A;
    arm_mat_init_f32(&A, M, N, A_buf);

    memcpy(A_buf, A_in->pData, sizeof(float)*M*N);
    for (uint32_t i = 0; i < M; i++)
        b_buf[i] = b_in->pData[i];

    for (uint32_t j = 0; j < N; j++) {
        col_perm[j] = j;
        col_norms[j] = col_norm_f32(&A, j, 0);
    }

    /* QR 分解（列主元） */
    for (uint32_t k = 0; k < N; k++) {

        uint32_t max_col = k;
        float max_norm = col_norms[k];
        for (uint32_t j = k + 1; j < N; j++) {
            if (col_norms[j] > max_norm) {
                max_norm = col_norms[j];
                max_col = j;
            }
        }

        if (max_col != k) {
            swap_cols_f32(&A, k, max_col);
            swap_int(&col_perm[k], &col_perm[max_col]);
            float t = col_norms[k];
            col_norms[k] = col_norms[max_col];
            col_norms[max_col] = t;
        }

        int len = M - k;
        for (int i = 0; i < len; i++)
            v[i] = MAT(&A, k + i, k);

        float beta = compute_householder(v, len);

        if (beta != 0.0f) {
            for (uint32_t j = k; j < N; j++) {
                float w = 0.0f;
                for (int i = 0; i < len; i++)
                    w += v[i] * MAT(&A, k + i, j);

                w *= beta;
                for (int i = 0; i < len; i++)
                    MAT(&A, k + i, j) -= w * v[i];
            }

            float wb = 0.0f;
            for (int i = 0; i < len; i++)
                wb += v[i] * b_buf[k + i];

            wb *= beta;
            for (int i = 0; i < len; i++)
                b_buf[k + i] -= wb * v[i];
        }
    }

    /* 回代 */
    float z[3];
    for (int i = N - 1; i >= 0; i--) {
        float sum = 0.0f;
        for (uint32_t j = i + 1; j < N; j++)
            sum += MAT(&A, i, j) * z[j];

        float diag = MAT(&A, i, i);
        if (fabsf(diag) < 1e-12f)
            return -1;

        z[i] = (b_buf[i] - sum) / diag;
    }

    /* 恢复列顺序 */
    for (uint32_t i = 0; i < N; i++)
        x_out->pData[col_perm[i]] = z[i];

    return 0;
}
