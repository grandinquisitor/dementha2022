/*
 * fxpt_atan2.c
 *
 * Copyright (C) 2012, Xo Wang
 *
 * Hacked up to be a bit more ARM-friendly by:
 * Copyright (C) 2013 Jared Boone, ShareBrained Technology, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef FXPT_ATAN2_H
#define FXPT_ATAN2_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Convert floating point to Q15 (1.0.15 fixed point) format.
 *
 * @param d floating-point value within range -1 to (1 - (2**-15)), inclusive
 * @return Q15 value representing d; same range
 */
/*
static inline int16_t q15_from_double(const double d) {
    return lrint(d * 32768);
}
*/
/**
 * Negative absolute value. Used to avoid undefined behavior for most negative
 * integer (see C99 standard 7.20.6.1.2 and footnote 265 for the description of
 * abs/labs/llabs behavior).
 *
 * @param i 16-bit signed integer
 * @return negative absolute value of i; defined for all values of i
 */
 /*
static inline int16_t s16_nabs(const int16_t j) {
#if (((int16_t)-1) >> 1) == ((int16_t)-1)
    // signed right shift sign-extends (arithmetic)
    const int16_t negSign = ~(j >> 15); // splat sign bit into all 16 and complement
    // if j is positive (negSign is -1), xor will invert j and sub will add 1
    // otherwise j is unchanged
    return (j ^ negSign) - negSign;
#else
    return (j < 0 ? j : -j);
#endif
}
*/
/**
 * Q15 (1.0.15 fixed point) multiplication. Various common rounding modes are in
 * the function definition for reference (and preference).
 *
 * @param j 16-bit signed integer representing -1 to (1 - (2**-15)), inclusive
 * @param k same format as j
 * @return product of j and k, in same format
 */
static inline int32_t q15_mul(const int32_t j, const int32_t k);

/**
 * Q15 (1.0.15 fixed point) division (non-saturating). Be careful when using
 * this function, as it does not behave well when the result is out-of-range.
 *
 * Value is not defined if numerator is greater than or equal to denominator.
 *
 * @param numer 16-bit signed integer representing -1 to (1 - (2**-15))
 * @param denom same format as numer; must be greater than numerator
 * @return numer / denom in same format as numer and denom
 */
static int32_t q15_div(const int32_t numer, const int32_t denom);

/**
 * 16-bit fixed point four-quadrant arctangent. Given some Cartesian vector
 * (x, y), find the angle subtended by the vector and the positive x-axis.
 *
 * The value returned is in units of 1/65536ths of one turn. This allows the use
 * of the full 16-bit unsigned range to represent a turn. e.g. 0x0000 is 0
 * radians, 0x8000 is pi radians, and 0xFFFF is (65535 / 32768) * pi radians.
 *
 * Because the magnitude of the input vector does not change the angle it
 * represents, the inputs can be in any signed 16-bit fixed-point format.
 *
 * @param y y-coordinate in signed 16-bit
 * @param x x-coordinate in signed 16-bit
 * @return angle in (val / 32768) * pi radian increments from 0x0000 to 0xFFFF
 */
uint16_t fxpt_atan2(const int32_t y, const int32_t x);


#ifdef __cplusplus
}
#endif

#endif /* FXPT_ATAN2_H */
