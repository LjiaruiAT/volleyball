#ifndef __SVD_H__
#define __SVD_H__

#include "arm_math.h"
#include <math.h>

arm_status arm_mat_svd_f32(
    const arm_matrix_instance_f32 *pSrc,
    arm_matrix_instance_f32 *pU,
    float32_t *pS,
    arm_matrix_instance_f32 *pVT,
    float32_t tol,
    uint32_t maxSweeps);

	arm_status arm_mat_min_norm_solve_f32(
    const arm_matrix_instance_f32 *pA,
    const arm_matrix_instance_f32 *pB,
    arm_matrix_instance_f32 *pX,
    float32_t tol);

#endif