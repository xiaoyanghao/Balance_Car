#ifndef __TXG_LOW_PASS_FILTER_H
#define __TXG_LOW_PASS_FILTER_H

/** Second order low pass filter structure.
 *
 * using biquad filter with bilinear z transform
 *
 * http://en.wikipedia.org/wiki/Digital_biquad_filter
 * http://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform
 *
 * Laplace continious form:
 *
 *                 1
 * H(s) = -------------------
 *        s^2/w^2 + s/w*Q + 1
 *
 *
 * Polynomial discrete form:
 *
 *        b0 + b1 z^-1 + b2 z^-2
 * H(z) = ----------------------
 *        a0 + a1 z^-1 + a2 z^-2
 *
 * with:
 *  a0 = 1
 *  a1 = 2*(K^2 - 1) / (K^2 + K/Q + 1)
 *  a2 = (K^2 - K/Q + 1) / (K^2 + K/Q + 1)
 *  b0 = K^2 / (K^2 + K/Q + 1)
 *  b1 = 2*b0
 *  b2 = b0
 *  K = tan(pi*Fc/Fs) ~ pi*Fc/Fs = Ts/(2*tau)
 *  Fc: cutting frequency
 *  Fs: sampling frequency
 *  Ts: sampling period
 *  tau: time constant
 */
struct SecondOrderLowPass_t
{
    double a[2]; ///< denominator gains
    double b[2]; ///< numerator gains
    double i[2]; ///< input history
    double o[2]; ///< output history
};

struct FirstOrderLowPass_t
{
    double a[2]; ///< denominator gains
    double b[2]; ///< numerator gains
    double i[2]; ///< input history
    double o[2]; ///< output history
};
/** Fourth order Butterworth low pass filter.
 *
 * using two cascaded second order filters
 */
struct Butterworth4LowPass_t
{
    struct SecondOrderLowPass_t lp1;
    struct SecondOrderLowPass_t lp2;
};

struct Butterworth6LowPass_t
{
    struct SecondOrderLowPass_t lp_6th_1;
    struct SecondOrderLowPass_t lp_6th_2;
    struct SecondOrderLowPass_t lp_6th_3;
};

/** Function type declaration ******************************************************************* */
void init_second_order_low_pass(struct SecondOrderLowPass_t *filter, double f_cut, double f_s, double Q, double value);
double update_second_order_low_pass(struct SecondOrderLowPass_t *filter, double value);
double get_second_order_low_pass(struct SecondOrderLowPass_t *filter);

void init_butterworth_4_low_pass(struct Butterworth4LowPass_t *filter, double f_cut, double f_s, double value);
double update_butterworth_4_low_pass(struct Butterworth4LowPass_t *filter, double value);
double get_butterworth_4_low_pass(struct Butterworth4LowPass_t *filter);

void init_butterworth_6_low_pass(struct Butterworth6LowPass_t *filter, double f_cut, double f_s, double value);
double update_butterworth_6_low_pass(struct Butterworth6LowPass_t *filter, double value);

void init_butterworth_first_order_low_pass(struct FirstOrderLowPass_t *filter, double f_cut, double f_s, double value);
double update_butterworth_first_order_low_pass(struct FirstOrderLowPass_t *filter, double value);
#endif

/** END OF FILE ********************************************************************************* */
