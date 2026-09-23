#ifndef __TXG_VECTOR3_H
#define __TXG_VECTOR3_H

#include <stdint.h>
#include <stdbool.h>

/** External structure ************************************************************************** */
/** Vector 2 double */
__packed struct vector2d_t
{
    double	x;
    double	y;
};

/** Vector 2 float */
__packed struct vector2f_t
{
    double	x;				// x element of vector
    double	y;           	// y
};

/** Vector 3 float */
__packed struct vector3f_t
{
    double	x;
    double	y;
    double	z;
};

/** Vector 2 int16 */
__packed struct vector2s_t
{
    int16_t x;
    int16_t y;
};

/** Vector 3 int16 */
__packed struct vector3s_t
{
    int16_t	x;
    int16_t y;
    int16_t z;
};

/** External functions ************************************************************************** */
// 2D vector length
double pythagorous2(double a, double b);
// 3D vector length
double pythagorous3(double a, double b, double c);
// vector cross product
void vector_cross_product(double vec1[3],double vec2[3], double out[3]);
// dot product
double vector_dot_product(double vector[3],double v[3]);
// vector3 length
double vector_length(double vector[3]);
// *=
void vector_product_equal(double vector[3], double num);
// /=
void vector_div_equal(double vector[3], double num);
// -=
void vector_minus_equal(double vector[3], double v[3]);
// +=
void vector_add_equal(double vector[3], double v[3]);

// vector nan/inf check
bool vector_is_nan(double vector[3]);
bool vector_is_inf(double vector[3]);

///vector divide a number
void vector_div(double vector[3], double num,double out[3]);
/// vector product a number
void vector_product(double vector[3], double num,double out[3]);
///vector minus a vector
void vector_minus(double vector[3],double v[3],double out[3]);
/// vector add by a vector
void vector_add(double vector[3],double v[3],double out[3]);
/// negative of the vector
void vector_negative(double vector[3],double out[3]);
/// vector equal
bool vector_equal(double vector[3],double v[3]);
/// vector not equal
bool vector_not_equal(double vector[3],double v[3]);
// normalizes this vector
void vector_normalize(double vector[3]);
// zero the vector
void vector_zero(double vector[3]);
bool is_zero(double vector[3]);


#endif // VECTOR3_H
