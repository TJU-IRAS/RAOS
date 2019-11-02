#include <string.h>
#include "lapacke.h"

// solve x from Ax=b
void solve_linear_equations(int num_eqs, int num_columns_b, float *A, float *x, float *b)
{
    int *ipiv = new int[num_eqs * num_eqs];
    float *A_cpy = new float[num_eqs * num_eqs];
    memcpy(A_cpy, A, sizeof(float) * num_eqs * num_eqs);
    float *b_cpy = new float[num_eqs * num_columns_b];
    memcpy(b_cpy, b, sizeof(float) * num_eqs * num_columns_b);
    LAPACKE_sgesv(LAPACK_ROW_MAJOR, num_eqs, num_columns_b, A_cpy, num_eqs, ipiv, b_cpy, num_columns_b);
    memcpy(x, b_cpy, sizeof(float) * num_eqs * num_columns_b);
    delete[] ipiv;
    delete[] A_cpy;
    delete[] b_cpy;
}
