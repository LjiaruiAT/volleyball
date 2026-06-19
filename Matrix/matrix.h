#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <stdio.h>
#include <math.h>
#include <arm_math.h>


int solve_linear_system_qr_f32(
    const arm_matrix_instance_f32 *A_in,
    const arm_matrix_instance_f32 *b_in,
    arm_matrix_instance_f32 *x_out);


#endif