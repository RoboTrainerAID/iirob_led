#ifndef IIROB_LED_MATH_H
#define IIROB_LED_MATH_H

/**
 * @brief scalingFactor returns the scaling factor for converting an element from one range to another
 * @param rangeOld_min The minimum value of the first (old) range (for example 0 in [0..10])
 * @param rangeOld_max The maximum value of the first (old) range (for example 10 in [0..10])
 * @param rangeNew_min The minimum value of the second (new) range (for example 0 in [0..30])
 * @param rangeNew_max The maximum value of the second (new) range (for example 30 in [0..30])
 * @return scaling factor which can then be used to map a value from the first (old) to the second (new) range
 */
double scalingFactor(double rangeOld_min, double rangeOld_max, double rangeNew_min, double rangeNew_max);

/**
 * @brief convert Converts a value given a scaling function
 * @param scalingFun See @scalingFactor (range is defined by @rangeOld_min, @rangeOld_max, @rangeNew_min and @rangeNew_max
 * @param rangeOld_min The minimum value of the first (old) range (for example 0 in [0..10])
 * @param rangeOld_max The maximum value of the first (old) range (for example 10 in [0..10])
 * @param rangeNew_min The minimum value of the second (new) range (for example 0 in [0..30])
 * @param rangeNew_max The maximum value of the second (new) range (for example 30 in [0..30])
 * @param value Number that we want to map from the first (old) to the second (new) range
 * @return scaled @value from the first (old) to the second (new) range
 */
double convert(double (*scalingFun)(double, double, double, double), double rangeOld_min, double rangeOld_max, double rangeNew_min, double rangeNew_max, double value);

/**
 * @brief convert Converts a value given a scaling factor (result from a scaling function)
 * @param scalingFactor Precalculated scaling factor
 * @param rangeOld_min The minimum value of the first (old) range (for example 0 in [0..10])
 * @param rangeNew_min The minimum value of the second (new) range (for example 0 in [0..30])
 * @param value Number that we want to map from the first (old) to the second (new) range
 * @return scaled @value from the first (old) to the second (new) range
 */
double convert(double scalingFactor, double rangeOld_min, double rangeNew_min, double value);

#endif // IIROB_LED_RECTANGLE_H
