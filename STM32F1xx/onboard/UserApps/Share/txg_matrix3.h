#ifndef MATRIX3_H
#define MATRIX3_H

//#include "txg_define.h"
#include <stdint.h>
#include "txg_vector3.h"

// ׾ޗߘֳ
__packed struct matrix2f_t
{
    struct vector2f_t	a;
    struct vector2f_t	b;
};

// ɽޗߘֳ
__packed struct matrix3f_t
{
    struct vector3f_t	a;
    struct vector3f_t	b;
    struct vector3f_t	c;
};

// create a rotation matrix given some euler angles
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
void matrix_from_euler(double matrix[3][3], double roll, double pitch, double yaw);
// calculate euler angles from a rotation matrix
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
void matrix_to_euler(double matrix[3][3],double *roll, double *pitch, double *yaw);
// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
void matrix_rotate(double matrix[3][3],double g[3]);
// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
void matrix_rotateXY(double matrix[3][3],double g[3]);
// apply an additional inverse rotation to a rotation matrix but
// only use X, Y elements from rotation vector
void matrix_rotateXYinv(double matrix[3][3],double g[3]);
// multiplication by a vector
void matrix_multi_vector(double matrix[3][3],double v[3],double out[3]);
// multiplication by a vector, extracting only the xy components
void matrix_mulXY(double matrix[3][3],double v[3],double out[2]);
// multiplication of transpose by a vector
void matrix_mul_transpose(double matrix[3][3],double v[3],double out[3]);
// multiplication by another Matrix3<T>
void matrix_multi_matrix(double matrix[3][3],double m[3][3],double out[3][3]);

void matrix_transposed(double matrix[3][3]);

void matrix_zero(double matrix[3][3]);
// extract x column
void colx_of(double matrix[3][3],double out[3]);
// extract y column
void coly_of(double matrix[3][3],double out[3]);
// extract z column
void colz_of(double matrix[3][3],double out[3]);
// negation
//void matrix_negation(double matrix[3][3],double out[3][3]);
// addition
//void matrix_addition(double matrix[3][3],double m[3][3],double out[3][3]);
// add equal
//void matrix_add_equal(double matrix[3][3],double m[3][3]);
// subtraction
//void matrix_subtraction(double matrix[3][3],double m[3][3],double out[3][3]);
// subtraction equal
//void matrix_minus_equal(double matrix[3][3],double m[3][3]);
// uniform scaling
//void matrix_product_num(double matrix[3][3],double num,double out[3][3]);
// product equal
//void matrix_product_equal(double matrix[3][3],double num);
// divide scaling
//void matrix_div_num(double matrix[3][3],double num,double out[3][3]);
// divide equal
//void matrix_div_equal(double matrix[3][3],double num);
// matrix product equal
//void matrix_matrix_product_equal(double matrix[3][3],double m[3][3]);
//void transpose(double matrix[3][3]);
// setup the identity matrix
//void identity(double matrix[3][3]);
// check if any elements are NAN
bool matrix_is_nan(double matrix[3][3]);

void matrix_transpose(double* A, int8_t m, int8_t n, double* C);
void matrix_multiply(double* A, double* B, int8_t m, int8_t p, int8_t n, double* C);
int8_t matrix_inversion(double* A, int8_t n, double* AInverse);

#endif
