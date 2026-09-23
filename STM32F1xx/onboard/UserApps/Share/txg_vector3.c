#include "txg_vector3.h"
#include "txg_math.h"
#include <math.h>


/**
 * Length of 2D vector.
 * @param  a : x part of 2D vector
 * @param  b : y part of 2D vector
 * @return   : length of vector
 */
double pythagorous2(double a, double b)
{
    return safe_sqrt(a*a+b*b);
}

/**
 * Length of 3D vector
 * @param  a : x part
 * @param  b : y part
 * @param  c : z part
 * @return   : length
 */
double pythagorous3(double a, double b, double c)
{
    return safe_sqrt(a*a+b*b+c*c);
}

/**
 * Cross product of two vector
 * @param  vec1: vector a
 * @param  vec2: vector b
 * @param   out: cross procduct output
 */
void vector_cross_product(double vec1[3],double vec2[3], double out[3])
{
    out[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1];
    out[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2];
    out[2] = vec1[0]*vec2[1] - vec1[1]*vec2[0];
}

/**
 * Dot product.
 * @param vec1: vec a
 * @param vec2: vec b
 * @return    : ret of dot product
 */
double vector_dot_product(double vec1[3],double vec2[3])
{
    return (vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2]);
}


double vector_length(double vector[3])
{
    return pythagorous3(vector[0], vector[1], vector[2]);
}

//*=
void vector_product_equal(double vector[3], double num)
{
    vector[0] *= num;
    vector[1] *= num;
    vector[2] *= num;
}

// /=
void vector_div_equal(double vector[3], double num)	//是否要对除数做非0判断	//marked by wgs
{
    if (!is_zero_D(num))
    {
        vector[0] /= num;
        vector[1] /= num;
        vector[2] /= num;
    }
    else
    {
        vector[0] = 0.0f;
        vector[1] = 0.0f;
        vector[2] = 0.0f;
    }
}

// -=
void vector_minus_equal(double vector[3], double v[3])
{
    vector[0] -= v[0];
    vector[1] -= v[1];
    vector[2] -= v[2];
}

// +=
void vector_add_equal(double vector[3], double v[3])
{
    vector[0] += v[0];
    vector[1] += v[1];
    vector[2] += v[2];
}

// nan check
bool vector_is_nan(double vector[3])
{
    return isnan(vector[0]) || isnan(vector[1]) || isnan(vector[2]);
}

// inf check
bool vector_is_inf(double vector[3])
{
    return isinf(vector[0]) || isinf(vector[1]) || isinf(vector[2]);
}

///vector divide a number
void vector_div(double vector[3], double num, double out[3])	//是否要对除数做非0判断	//marked by wgs
{
    if (!is_zero_D(num))
    {
        out[0] = vector[0] / num;
        out[1] = vector[1] / num;
        out[2] = vector[2] / num;
    }
    else
    {
        out[0] = 0.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
    }
}

/// vector product a number
void vector_product(double vector[3], double num,double out[3])
{
    out[0] = vector[0] * num;
    out[1] = vector[1] * num;
    out[2] = vector[2] * num;
}

///vector minus a vector
void vector_minus(double vector[3],double v[3],double out[3])
{
    out[0] = vector[0] -v[0];
    out[1] = vector[1] -v[1];
    out[2] = vector[2] -v[2];
}

/// vector add by a vector
void vector_add(double vector[3],double v[3],double out[3])
{
    out[0] = vector[0] +v[0];
    out[1] = vector[1] +v[1];
    out[2] = vector[2] +v[2];
}

/// negative of the vector
void vector_negative(double vector[3],double out[3])
{
    out[0] = -vector[0];
    out[1] = -vector[1];
    out[2] = -vector[2];
}

/// vector equal
bool vector_equal(double vector[3],double v[3])
{
    // return (vector[0]==v[0] && vector[1]==v[1] && vector[2]==v[2]);	//double类型的相等判断是否应调用函数is_equal_D()	//marked by wgs

    bool equal_ret = is_equal_D(vector[0], v[0]) & is_equal_D(vector[1], v[1]) & is_equal_D(vector[2], v[2]);

    return equal_ret;
}

/// vector not equal
bool vector_not_equal(double vector[3],double v[3])
{
    // return (vector[0]!=v[0] && vector[1]!=v[1] && vector[2]!=v[2]);	//double类型的相等判断是否应调用函数is_equal_D()	//marked by wgs

    bool equal_ret = (!is_equal_D(vector[0], v[0])) & (!is_equal_D(vector[1], v[1])) & (!is_equal_D(vector[2], v[2]));

    return equal_ret;
}

// normalizes this vector
void vector_normalize(double vector[3])	//是否要对除数做非0判断	//marked by wgs
{
    double a = vector_length(vector);
    vector[0] /=a;
    vector[1] /=a;
    vector[2] /=a;
}

// zero the vector
void vector_zero(double vector[3])
{
    vector[0] = 0;
    vector[1] = 0;
    vector[2] = 0;
}

bool is_zero(double vector[3])
{
    return (is_zero_D(vector[0]) && is_zero_D(vector[1]) && is_zero_D(vector[2]));
}

