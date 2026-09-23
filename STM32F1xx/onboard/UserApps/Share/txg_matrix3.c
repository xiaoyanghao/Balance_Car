#include "txg_matrix3.h"
#include <math.h>
#include <stdlib.h>



#ifndef PI
#define PI 3.14159265
#endif

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
double safe_asin(double v)
{
    if (isnan(v))
    {
        return 0.0;
    }
    if (v >= 1.0)
    {
        return PI/2;
    }
    if (v <= -1.0)
    {
        return -PI/2;
    }
    return asinf(v);
}


// create a rotation matrix given some euler angles
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
void matrix_from_euler(double matrix[3][3], double roll, double pitch, double yaw)
{
    double cp = cosf(pitch);
    double sp = sinf(pitch);
    double sr = sinf(roll);
    double cr = cosf(roll);
    double sy = sinf(yaw);
    double cy = cosf(yaw);

    //transition matrix from ned frame to body frame
    matrix[0][0]= cp * cy;
    matrix[0][1] = (sr * sp * cy) - (cr * sy);
    matrix[0][2] = (cr * sp * cy) + (sr * sy);
    matrix[1][0] = cp * sy;
    matrix[1][1] = (sr * sp * sy) + (cr * cy);
    matrix[1][2] = (cr * sp * sy) - (sr * cy);
    matrix[2][0] = -sp;
    matrix[2][1] = sr * cp;
    matrix[2][2] = cr * cp;
}

// calculate euler angles from a rotation matrix
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
void matrix_to_euler(double matrix[3][3],double* roll, double* pitch, double* yaw)
{
    *pitch = -safe_asin(matrix[2][0]);
    *roll = atan2f(matrix[2][1], matrix[2][2]);
    *yaw = atan2f(matrix[1][0], matrix[0][0]);
}

// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
void matrix_rotate(double matrix[3][3],double g[3])
{
    double temp_matrix[3][3];
    temp_matrix[0][0] = matrix[0][1] * g[2] - matrix[0][2] * g[1];
    temp_matrix[0][1] = matrix[0][2] * g[0] - matrix[0][0] * g[2];
    temp_matrix[0][2] = matrix[0][0] * g[1] - matrix[0][1] * g[0];
    temp_matrix[1][0] = matrix[1][1] * g[2] - matrix[1][2] * g[1];
    temp_matrix[1][1] = matrix[1][2] * g[0] - matrix[1][0] * g[2];
    temp_matrix[1][2] = matrix[1][0] * g[1] - matrix[1][1] * g[0];
    temp_matrix[2][0] = matrix[2][1] * g[2] - matrix[2][2] * g[1];
    temp_matrix[2][1] = matrix[2][2] * g[0] - matrix[2][0] * g[2];
    temp_matrix[2][2] = matrix[2][0] * g[1] - matrix[2][1] * g[0];

    for (int8_t i=0; i<3; i++)
        for (int8_t j=0; j<3; j++)
            matrix[i][j] += temp_matrix[i][j];
}

//// apply an additional rotation from a body frame gyro vector
//// to a rotation matrix.
//void matrix_rotateXY(double matrix[3][3],double g[3])
//{
//    double temp_matrix[3][3];
//    temp_matrix[0][0] = -matrix[0][2] * g[1];
//    temp_matrix[0][1] = matrix[0][2] * g[0];
//    temp_matrix[0][2] = matrix[0][0] * g[1] - matrix[0][1] * g[0];
//    temp_matrix[1][0] = -matrix[1][2] * g[1];
//    temp_matrix[1][1] = matrix[1][2] * g[0];
//    temp_matrix[1][2] = matrix[1][0] * g[1] - matrix[1][1] * g[0];
//    temp_matrix[2][0] = -matrix[2][2] * g[1];
//    temp_matrix[2][1] = matrix[2][2] * g[0];
//    temp_matrix[2][2] = matrix[2][0] * g[1] - matrix[2][1] * g[0];
//
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          matrix[i][j] += temp_matrix[i][j];
//}
//
//// apply an additional inverse rotation to a rotation matrix but
//// only use X, Y elements from rotation vector
//void matrix_rotateXYinv(double matrix[3][3],double g[3])
//{
//    double temp_matrix[3][3];
//    temp_matrix[0][0] =   matrix[0][2] * g[1];
//    temp_matrix[0][1] = - matrix[0][2] * g[0];
//    temp_matrix[0][2] = - matrix[0][0] * g[1] + matrix[0][1] * g[0];
//    temp_matrix[1][0] =   matrix[1][2] * g[1];
//    temp_matrix[1][1] = - matrix[1][2] * g[0];
//    temp_matrix[1][2] = - matrix[1][0] * g[1] + matrix[1][1] * g[0];
//    temp_matrix[2][0] =   matrix[2][2] * g[1];
//    temp_matrix[2][1] = - matrix[2][2] * g[0];
//    temp_matrix[2][2] = - matrix[2][0] * g[1] + matrix[2][1] * g[0];
//
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          matrix[i][j] += temp_matrix[i][j];
//}

// multiplication by a vector
void matrix_multi_vector(double matrix[3][3],double v[3],double out[3])
{
    out[0] = matrix[0][0] * v[0] + matrix[0][1] * v[1] + matrix[0][2] * v[2];
    out[1] = matrix[1][0] * v[0] + matrix[1][1] * v[1] + matrix[1][2] * v[2];
    out[2] = matrix[2][0] * v[0] + matrix[2][1] * v[1] + matrix[2][2] * v[2];
}

// multiplication by a vector, extracting only the xy components
void matrix_mulXY(double matrix[3][3],double v[3],double out[2])
{
    out[0] = matrix[0][0] * v[0] + matrix[0][1] * v[1] + matrix[0][2] * v[2];
    out[1] = matrix[1][0] * v[0] + matrix[1][1] * v[1] + matrix[1][2] * v[2];
}

// multiplication of transpose by a vector
void matrix_mul_transpose(double matrix[3][3],double v[3],double out[3])
{
    double temp[3];
    temp[0] = matrix[0][0] * v[0] + matrix[1][0] * v[1] + matrix[2][0] * v[2];
    temp[1] = matrix[0][1] * v[0] + matrix[1][1] * v[1] + matrix[2][1] * v[2];
    temp[2] = matrix[0][2] * v[0] + matrix[1][2] * v[1] + matrix[2][2] * v[2];

    for (int8_t i=0; i<3; i++)
        out[i] = temp[i];
}

// multiplication by another Matrix3<T>
void matrix_multi_matrix(double matrix[3][3],double m[3][3],double out[3][3])
{
    out[0][0] = matrix[0][0] * m[0][0] + matrix[0][1] * m[1][0] + matrix[0][2] * m[2][0];
    out[0][1] = matrix[0][0] * m[0][1] + matrix[0][1] * m[1][1] + matrix[0][2] * m[2][1];
    out[0][2] = matrix[0][0] * m[0][2] + matrix[0][1] * m[1][2] + matrix[0][2] * m[2][1];
    out[1][0] = matrix[1][0] * m[0][0] + matrix[1][1] * m[1][0] + matrix[1][2] * m[2][0];
    out[1][1] = matrix[1][0] * m[0][1] + matrix[1][1] * m[1][1] + matrix[1][2] * m[2][1];
    out[1][2] = matrix[1][0] * m[0][2] + matrix[1][1] * m[1][2] + matrix[1][2] * m[2][2];
    out[2][0] = matrix[2][0] * m[0][0] + matrix[2][1] * m[1][0] + matrix[2][2] * m[2][0];
    out[2][1] = matrix[2][0] * m[0][1] + matrix[2][1] * m[1][1] + matrix[2][2] * m[2][1];
    out[2][2] = matrix[2][0] * m[0][2] + matrix[2][1] * m[1][2] + matrix[2][2] * m[2][2];
}

void matrix_transposed(double matrix[3][3])
{
    double temp_matrix[3][3];

    temp_matrix[0][0] = matrix[0][0];
    temp_matrix[0][1] = matrix[1][0];
    temp_matrix[0][2] = matrix[2][0];
    temp_matrix[1][0] = matrix[0][1];
    temp_matrix[1][1] = matrix[1][1];
    temp_matrix[1][2] = matrix[2][1];
    temp_matrix[2][0] = matrix[0][2];
    temp_matrix[2][1] = matrix[1][2];
    temp_matrix[2][2] = matrix[2][2];

    for (int8_t i=0; i<3; i++)
        for (int8_t j=0; j<3; j++)
            matrix[i][j] = temp_matrix[i][j];
}

void matrix_zero(double matrix[3][3])
{
    matrix[0][0] = matrix[0][1] = matrix[0][2] = 0;
    matrix[1][0] = matrix[1][1] = matrix[1][2] = 0;
    matrix[2][0] = matrix[2][1] = matrix[2][2] = 0;
}

// extract x column
void colx_of(double matrix[3][3],double out[3])
{
    out[0] = matrix[0][0];
    out[1] = matrix[1][0];
    out[2] = matrix[2][0];
}

// extract y column
void coly_of(double matrix[3][3],double out[3])
{
    out[0] = matrix[0][1];
    out[1] = matrix[1][1];
    out[2] = matrix[2][1];
}

// extract z column
void colz_of(double matrix[3][3],double out[3])
{
    out[0] = matrix[0][2];
    out[1] = matrix[1][2];
    out[2] = matrix[2][2];
}


// negation
//void matrix_negation(double matrix[3][3],double out[3][3])
//{
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          out[i][j] = -matrix[i][j];
//}

// addition
//void matrix_addition(double matrix[3][3],double m[3][3],double out[3][3])
//{
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          out[i][j] = matrix[i][j] + m[i][j];
//}

// add equal
//void matrix_add_equal(double matrix[3][3],double m[3][3])
//{
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          matrix[i][j] += m[i][j];
//}
// subtraction
//void matrix_subtraction(double matrix[3][3],double m[3][3],double out[3][3])
//{
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          out[i][j] = matrix[i][j] - m[i][j];
//}
// subtraction equal
//void matrix_minus_equal(double matrix[3][3],double m[3][3])
//{
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          matrix[i][j] -= m[i][j];
//}
// uniform scaling
//void matrix_product_num(double matrix[3][3],double num,double out[3][3])
//{
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          out[i][j] = matrix[i][j]*num;
//}
// product equal
//void matrix_product_equal(double matrix[3][3],double num)
//{
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          matrix[i][j] = matrix[i][j]*num;
//}

// divide scaling
//void matrix_div_num(double matrix[3][3],double num,double out[3][3])
//{
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          out[i][j] = matrix[i][j]/num;
//}
// divide equal
//void matrix_div_equal(double matrix[3][3],double num)
//{
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          matrix[i][j] /= num;
//}

// matrix product equal
//void matrix_matrix_product_equal(double matrix[3][3],double m[3][3])
//{
//    double temp_matrix[3][3];
//    matrix_multi_matrix(matrix,m,temp_matrix);
//
//    for(int8_t i=0;i<3;i++)
//       for(int8_t j=0;j<3;j++)
//          matrix[i][j] = temp_matrix[i][j];
//}


//void transpose(double matrix[3][3])
//{
//    matrix_transposed(matrix);
//}

//// setup the identity matrix
//void identity(double matrix[3][3])
//{
//    matrix[0][0] = matrix[1][1] = matrix[2][2] = 1;
//    matrix[0][1] = matrix[0][2] = 0;
//    matrix[1][0] = matrix[1][2] = 0;
//    matrix[2][0] = matrix[2][1] = 0;
//}

// check if any elements are NAN
bool matrix_is_nan(double matrix[3][3])
{
    double temp_vector1[3],temp_vector2[3],temp_vector3[3];

    colx_of(matrix,temp_vector1);
    coly_of(matrix,temp_vector2);
    colz_of(matrix,temp_vector3);

    return vector_is_nan(temp_vector1) || vector_is_nan(temp_vector2) || vector_is_nan(temp_vector3);
}

/*******************************************************************************
* Function Name  : void matrix_transpose(double* A, int8_t m, int8_t n, double* C)
* Description    : 矩阵求转置
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void matrix_transpose(double* A, int8_t m, int8_t n, double* C)
// Matrix Transpose Routine
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = the transpose of A (n x m)
    int8_t i, j;
    for (i=0; i<m; i++)
        for (j=0; j<n; j++)
            C[m*j+i]=A[n*i+j];
}


/*******************************************************************************
* Function Name  : int8_t matrix_inversion(double* A, int8_t n, double* AInverse)
* Description    : 矩阵求逆
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int8_t matrix_inversion(double* A, int8_t n, double* AInverse)
// Matrix Inversion Routine
{
    // A = input matrix (n x n)
    // n = dimension of A
    // AInverse = inverted matrix (n x n)
    // This function inverts a matrix based on the Gauss Jordan method.
    // The function returns 1 on success, 0 on failure.
    int8_t i, j, iPass, imx, icol, irow;
    double det, temp, pivot, factor;
    double* ac = (double*)calloc(n*n, sizeof(double));
    det = 1;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            AInverse[n*i+j] = 0;
            ac[n*i+j] = A[n*i+j];
        }
        AInverse[n*i+i] = 1;
    }

    // The current pivot row is iPass.
    // For each pass, Frst Fnd the maximum element in the pivot column.
    for (iPass = 0; iPass < n; iPass++)
    {
        imx = iPass;
        for (irow = iPass; irow < n; irow++)
        {
            if (fabs(A[n*irow+iPass]) > fabs(A[n*imx+iPass])) imx = irow;
        }

        // Interchange the elements of row iPass and row imx in both A and AInverse.
        if (imx != iPass)
        {
            for (icol = 0; icol < n; icol++)
            {
                temp = AInverse[n*iPass+icol];
                AInverse[n*iPass+icol] = AInverse[n*imx+icol];
                AInverse[n*imx+icol] = temp;
                if (icol >= iPass)
                {
                    temp = A[n*iPass+icol];
                    A[n*iPass+icol] = A[n*imx+icol];
                    A[n*imx+icol] = temp;
                }
            }
        }

        // The current pivot is now A[iPass][iPass].
        // The determinant is the product of the pivot elements.
        pivot = A[n*iPass+iPass];
        det = det * pivot;
        if (det == 0)
        {
            free(ac);
            return 0;
        }

        for (icol = 0; icol < n; icol++)
        {
            // Normalize the pivot row by dividing by the pivot element.
            AInverse[n*iPass+icol] = AInverse[n*iPass+icol] / pivot;
            if (icol >= iPass) A[n*iPass+icol] = A[n*iPass+icol] / pivot;
        }

        for (irow = 0; irow < n; irow++)
        {
            // Add a multiple of the pivot row to each row.  The multiple factor
            // is chosen so that the element of A on the pivot column is 0.
            if (irow != iPass) factor = A[n*irow+iPass];
            for (icol = 0; icol < n; icol++)
            {
                if (irow != iPass)
                {
                    AInverse[n*irow+icol] -= factor * AInverse[n*iPass+icol];
                    A[n*irow+icol] -= factor * A[n*iPass+icol];
                }
            }
        }
    }

    free(ac);
    return 1;
}


/*******************************************************************************
* Function Name  : void matrix_multiply(double* A, double* B, int8_t m, int8_t p, int8_t n, double* C)
* Description    : 两个矩阵相乘
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void matrix_multiply(double* A, double* B, int8_t m, int8_t p, int8_t n, double* C)
// Matrix Multiplication Routine
{
    // A = input matrix (m x p)
    // B = input matrix (p x n)
    // m = number of rows in A
    // p = number of columns in A = number of rows in B
    // n = number of columns in B
    // C = output matrix = A*B (m x n)
    int8_t i, j, k;
    for (i=0; i<m; i++)
    {
        for (j=0; j<n; j++)
        {
            C[n*i+j]=0;
            for (k=0; k<p; k++)
                C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
        }
    }
}
/*******************************************************************************
* Function Name  :void matrix_addition(double* A, double* B, int8_t m, int8_t n, double* C)
* Description    : 两个矩阵相加
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void matrix_MxN_addition(double* A, double* B, int8_t m, int8_t n, double* C)
// Matrix Addition Routine
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A+B (m x n)
    int8_t i, j;
    for (i=0; i<m; i++)
        for (j=0; j<n; j++)
            C[n*i+j]=A[n*i+j]+B[n*i+j];
}
/*******************************************************************************
* Function Name  :void matrix_subtraction(double* A, double* B, int8_t m, int8_t n, double* C)
* Description    : 两个矩阵相减
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void matrix_MxN_subtraction(double* A, double* B, int8_t m, int8_t n, double* C)
// Matrix Subtraction Routine
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A-B (m x n)
    int8_t i, j;
    for (i=0; i<m; i++)
        for (j=0; j<n; j++)
            C[n*i+j]=A[n*i+j]-B[n*i+j];
}

/*******************************************************************************
* Function Name  :void matrix_subtraction(double* A, double* B, int8_t m, int8_t n, double* C)
* Description    : 两个矩阵相减
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void matrix_constant_multiply(double * A, int8_t m, int8_t n, double constant, double * C)
// Matrix Subtraction Routine
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A-B (m x n)
    int8_t i, j;
    for (i=0; i<m; i++)
        for (j=0; j<n; j++)
            C[n*i+j]=A[n*i+j]*constant;
}
