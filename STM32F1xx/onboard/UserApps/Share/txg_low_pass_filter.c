/*
* @ahthor: cmeng@topXgun.com
* @date: 2019/12/10
* @function: (1st / 2nd / 4th /6th) butterworth low pass filters' parameters calibration
*/

#include "txg_low_pass_filter.h"
#include <math.h>
#include "txg_math.h"
/** Init second order low pass filter.
 *
 * @param filter second order low pass filter structure
 * @param tau time constant of the second order low pass filter
 * @param Q Q value of the second order low pass filter, Q=0.707 for Butterworth
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
void init_second_order_low_pass(struct SecondOrderLowPass_t *filter, double f_cut, double f_s, double Q, double value)
{
    double K = tan(M_PI*f_cut/f_s);
    double poly = K * K + K / Q + 1.0;
    filter->a[0] = 2.0 * (K * K - 1.0) / poly;
    filter->a[1] = (K * K - K / Q + 1.0) / poly;
    filter->b[0] = K * K / poly;
    filter->b[1] = 2.0 * filter->b[0];
    filter->i[0] = filter->i[1] = filter->o[0] = filter->o[1] = value;
}

/** Update second order low pass filter state with a new value.
 *
 * @param filter second order low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
double update_second_order_low_pass(struct SecondOrderLowPass_t *filter, double value)
{
    double out = filter->b[0] * value	           // K input
                 + filter->b[1] * filter->i[0]              // K-1 input
                 + filter->b[0] * filter->i[1]          // K-2 input
                 - filter->a[0] * filter->o[0]      // K-1 output
                 - filter->a[1] * filter->o[1]; // K-2 output
    filter->i[1] = filter->i[0];
    filter->i[0] = value;
    filter->o[1] = filter->o[0];
    filter->o[0] = out;
    return out;
}

/** Get current value of the second order low pass filter.
 *
 * @param filter second order low pass filter structure
 * @return current value of the filter
 */
double get_second_order_low_pass(struct SecondOrderLowPass_t *filter)
{
    return filter->o[0];
}

/** Init a fourth order Butterworth filter.
 *
 * based on two generic second order filters
 * with Q1 = 1.30651
 *  and Q2 = 0.541184
 *
 * http://en.wikipedia.org/wiki/Butterworth_filter
 *
 * @param filter fourth order Butterworth low pass filter structure
 * @param tau time constant of the fourth order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
void init_butterworth_4_low_pass(struct Butterworth4LowPass_t *filter, double f_cut, double f_s, double value)
{
    init_second_order_low_pass(&filter->lp1, f_cut, f_s, 1.30651, value);
    init_second_order_low_pass(&filter->lp2, f_cut, f_s, 0.541184, value);
}

/** Update fourth order Butterworth low pass filter state with a new value.
 *
 * using two cascaded second order filters
 *
 * @param filter fourth order Butterworth low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
double update_butterworth_4_low_pass(struct Butterworth4LowPass_t *filter, double value)
{
    double tmp = update_second_order_low_pass(&filter->lp1, value);
    return update_second_order_low_pass(&filter->lp2, tmp);
}

/** Get current value of the fourth order Butterworth low pass filter.
 *
 * @param filter fourth order Butterworth low pass filter structure
 * @return current value of the filter
 */
double get_butterworth_4_low_pass(struct Butterworth4LowPass_t *filter)
{
    return filter->lp2.o[0];
}


/** Init a sixth order Butterworth filter.
 *
 * based on two generic second order filters
 * with Q1 = 1.93199
 *  and Q2 = 0.70711
 *  and Q3 = 0.51762
 *
 * http://en.wikipedia.org/wiki/Butterworth_filter
 *
 * @param filter sixth order Butterworth low pass filter structure
 * @param tau time constant of the sixrh order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 */
void init_butterworth_6_low_pass(struct Butterworth6LowPass_t *filter, double f_cut, double f_s, double value)
{
    init_second_order_low_pass(&filter->lp_6th_1, f_cut, f_s, 1.93199, value);
    init_second_order_low_pass(&filter->lp_6th_2, f_cut, f_s, 0.70711, value);
    init_second_order_low_pass(&filter->lp_6th_3, f_cut, f_s, 0.51762, value);
}

/** Update sixth order Butterworth low pass filter state with a new value.
 *
 * using three cascaded second order filters
 *
 * @param filter fourth order Butterworth low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
double update_butterworth_6_low_pass(struct Butterworth6LowPass_t *filter, double value)
{
    double tmp1 = update_second_order_low_pass(&filter->lp_6th_1, value);
    double tmp2 = update_second_order_low_pass(&filter->lp_6th_2, tmp1);
    return update_second_order_low_pass(&filter->lp_6th_3, tmp2);
}

/** Init first order low pass filter.
 *  Bilinear transformation
 * @param filter first order low pass filter structure
 * @param tau time constant of the first order low pass filter
 * @param sample_time sampling period of the signal
 * @param value initial value of the filter
 * Polynomial discrete form:
 *        b0 + b1 z^-1
 * H(z) = ------------
 *        a0 + a1 z^-1
 */
void init_butterworth_first_order_low_pass(struct FirstOrderLowPass_t *filter, double f_cut, double f_s, double value)
{
    double K = tan(M_PI*f_cut/f_s);
    filter->a[0] = (K - 1)/(K + 1.0);       // N-1 output
    filter->b[0] = K / (K + 1.0);           // N-1 input
    filter->i[0] = filter->o[0] = value;
}

/** Update first order low pass filter state with a new value.
 *
 * @param filter first order low pass filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
double update_butterworth_first_order_low_pass(struct FirstOrderLowPass_t *filter, double value)
{
    double out = filter->b[0] * value	                     // K input
                 + filter->b[0] * filter->i[0]              // K-1 input
                 - filter->a[0] * filter->o[0];       // K-1 output
    filter->i[0] = value;   // 1st_butterworth just record a history value
    filter->o[0] = out;
    return out;
}

/** END OF FILE ********************************************************************************* */
