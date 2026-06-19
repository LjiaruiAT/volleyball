#include "svd.h"


/**
 * @brief Compute thin Singular Value Decomposition (SVD) of a matrix using one-sided Jacobi iteration.
 *
 * @param[in]  pSrc        points to the input matrix instance (m x n)
 * @param[out] pU          points to the output left singular matrix instance (m x r), may be NULL
 * @param[out] pS          points to the output singular values (vector of length r)
 * @param[out] pVT         points to the output right singular vectors transposed (r x n), may be NULL
 * @param[in]  tol         convergence tolerance (e.g. 1e-7f)
 * @param[in]  maxSweeps   maximum Jacobi sweeps (e.g. 100)
 * @return     ARM_MATH_SUCCESS if converged, otherwise ARM_MATH_SINGULAR
 *
 * This function computes the thin SVD:
 *     A = U * diag(S) * VT,
 * where:
 *     - A is m x n
 *     - U is m x r
 *     - S is r x 1 (vector of singular values)
 *     - VT is r x n
 * and r = min(m, n)
 *
 * Internally uses one-sided Jacobi orthogonalization.
 */
arm_status arm_mat_svd_f32(
    const arm_matrix_instance_f32 *pSrc,
    arm_matrix_instance_f32 *pU,
    float32_t *pS,
    arm_matrix_instance_f32 *pVT,
    float32_t tol,
    uint32_t maxSweeps)
{
    if (pSrc == NULL || pS == NULL)
        return ARM_MATH_ARGUMENT_ERROR;

    uint32_t m = pSrc->numRows;
    uint32_t n = pSrc->numCols;
    uint32_t r = (m < n) ? m : n;

    if (tol <= 0.0f)
        tol = 1e-7f;
    if (maxSweeps == 0)
        maxSweeps = 100;

    const float32_t *A = pSrc->pData;
    float32_t *U = (pU) ? pU->pData : NULL;
    float32_t *VT = (pVT) ? pVT->pData : NULL;

    /* --- allocate working buffers on stack using VLAs (C99) --- */
    double B[m][n];      /* working copy of A in double precision */
    double V[n][n];      /* right singular vectors */
    double sv[n];        /* singular values (temp before sorting) */

    /* --- initialize --- */
    for (uint32_t i = 0; i < m; ++i)
        for (uint32_t j = 0; j < n; ++j)
            B[i][j] = (double)A[i * n + j];

    /* V = I */
    for (uint32_t i = 0; i < n; ++i)
        for (uint32_t j = 0; j < n; ++j)
            V[i][j] = (i == j) ? 1.0 : 0.0;

    /* --- one-sided Jacobi SVD --- */
    double tol2 = (double)tol * (double)tol;
    uint32_t sweep;
    uint8_t converged = 0;

    for (sweep = 0; sweep < maxSweeps; ++sweep) {
        double off = 0.0;
        for (uint32_t p = 0; p < n - 1; ++p) {
            for (uint32_t q = p + 1; q < n; ++q) {
                /* compute dot products */
                double aap = 0.0, aqq = 0.0, apq = 0.0;
                for (uint32_t i = 0; i < m; ++i) {
                    double bp = B[i][p];
                    double bq = B[i][q];
                    aap += bp * bp;
                    aqq += bq * bq;
                    apq += bp * bq;
                }
                off += apq * apq;
                if (fabs(apq) <= tol * sqrt(aap * aqq))
                    continue;

                /* compute Jacobi rotation */
                double beta = (aqq - aap);
                double gamma = 2.0 * apq;
                double zeta = beta / gamma;
                double t = (zeta >= 0.0)
                    ? (1.0 / (zeta + sqrt(1.0 + zeta * zeta)))
                    : (1.0 / (zeta - sqrt(1.0 + zeta * zeta)));
                double c = 1.0 / sqrt(1.0 + t * t);
                double s = t * c;

                /* apply rotation to B (columns p,q) */
                for (uint32_t i = 0; i < m; ++i) {
                    double bp = B[i][p];
                    double bq = B[i][q];
                    B[i][p] = c * bp - s * bq;
                    B[i][q] = s * bp + c * bq;
                }

                /* apply same rotation to V */
                for (uint32_t i = 0; i < n; ++i) {
                    double vp = V[i][p];
                    double vq = V[i][q];
                    V[i][p] = c * vp - s * vq;
                    V[i][q] = s * vp + c * vq;
                }
            }
        }

        if (off < tol2) { converged = 1; break; }
    }

    /* --- compute singular values (norms of columns of B) --- */
    for (uint32_t j = 0; j < n; ++j) {
        double norm2 = 0.0;
        for (uint32_t i = 0; i < m; ++i)
            norm2 += B[i][j] * B[i][j];
        sv[j] = sqrt(norm2);
    }

    /* --- sort descending by singular value --- */
    for (uint32_t i = 0; i < r; ++i) {
        uint32_t piv = i;
        double maxv = sv[i];
        for (uint32_t j = i + 1; j < n; ++j) {
            if (sv[j] > maxv) { maxv = sv[j]; piv = j; }
        }
        if (piv != i) {
            /* swap columns in B and V */
            for (uint32_t k = 0; k < m; ++k) {
                double tmp = B[k][i]; B[k][i] = B[k][piv]; B[k][piv] = tmp;
            }
            for (uint32_t k = 0; k < n; ++k) {
                double tmp = V[k][i]; V[k][i] = V[k][piv]; V[k][piv] = tmp;
            }
            double tmp = sv[i]; sv[i] = sv[piv]; sv[piv] = tmp;
        }
    }

    /* --- output S, U, VT --- */
    for (uint32_t i = 0; i < r; ++i)
        pS[i] = (float32_t)sv[i];

    if (pVT != NULL) {
        /* VT is r x n, row-major; row i = V_col_i^T */
        for (uint32_t i = 0; i < r; ++i)
            for (uint32_t j = 0; j < n; ++j)
                pVT->pData[i * n + j] = (float32_t)V[j][i];
    }

    if (pU != NULL) {
        /* U = normalized columns of B */
        for (uint32_t j = 0; j < r; ++j) {
            double sigma = sv[j];
            if (sigma > 1e-15) {
                double invs = 1.0 / sigma;
                for (uint32_t i = 0; i < m; ++i)
                    pU->pData[i * r + j] = (float32_t)(B[i][j] * invs);
            } else {
                for (uint32_t i = 0; i < m; ++i)
                    pU->pData[i * r + j] = 0.0f;
            }
        }
    }

    return converged ? ARM_MATH_SUCCESS : ARM_MATH_SINGULAR;
}


arm_status arm_mat_min_norm_solve_f32(
    const arm_matrix_instance_f32 *pA,
    const arm_matrix_instance_f32 *pB,
    arm_matrix_instance_f32 *pX,
    float32_t tol)
{
    if (!pA || !pB || !pX) return ARM_MATH_ARGUMENT_ERROR;

    uint32_t m = pA->numRows;
    uint32_t n = pA->numCols;
    uint32_t nrhs = pB->numCols;  /* handle multiple right-hand sides */

    if (pB->numRows != m || pX->numRows != n || pX->numCols != nrhs)
        return ARM_MATH_SIZE_MISMATCH;

    if (tol <= 0.0f) tol = 1e-6f;

    const uint32_t r = (m < n) ? m : n;

    /* --- Allocate stack working buffers using VLA (C99) --- */
    float32_t S[r];
    float32_t U[m * r];
    float32_t VT[r * n];

    arm_matrix_instance_f32 matU = { m, r, U };
    arm_matrix_instance_f32 matVT = { r, n, VT };

    /* --- Step 1: Compute SVD --- */
    arm_status st = arm_mat_svd_f32(pA, &matU, S, &matVT, tol * 0.1f, 100);
    if (st != ARM_MATH_SUCCESS)
        return st;

    /* --- Step 2: Compute y = U^T * b --- */
    float32_t UT[r * m];
    arm_matrix_instance_f32 matUT = { r, m, UT };
    /* UT = transpose(U) */
    for (uint32_t i = 0; i < r; ++i)
        for (uint32_t j = 0; j < m; ++j)
            UT[i * m + j] = U[j * r + i];

    float32_t Y[r * nrhs];
    arm_matrix_instance_f32 matY = { r, nrhs, Y };
    arm_mat_mult_f32(&matUT, pB, &matY);

    /* --- Step 3: scale by ¦²? (pseudo-inverse of singular values) --- */
    for (uint32_t i = 0; i < r; ++i) {
        float32_t sigma = S[i];
        float32_t invs = (fabsf(sigma) > tol * S[0]) ? (1.0f / sigma) : 0.0f;
        for (uint32_t j = 0; j < nrhs; ++j) {
            Y[i * nrhs + j] *= invs;
        }
    }

    /* --- Step 4: x = V * Y --- */
    /* recall: VT is r x n, so V = transpose(VT) */
    float32_t V[n * r];
    arm_matrix_instance_f32 matV = { n, r, V };
    for (uint32_t i = 0; i < n; ++i)
        for (uint32_t j = 0; j < r; ++j)
            V[i * r + j] = VT[j * n + i];

    arm_mat_mult_f32(&matV, &matY, pX);

    return ARM_MATH_SUCCESS;
}

