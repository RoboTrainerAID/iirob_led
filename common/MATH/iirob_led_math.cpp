#include "iirob_led_math.h"

double scalingFactor(double rangeOld_min, double rangeOld_max, double rangeNew_min, double rangeNew_max)
{
    return (rangeNew_max - rangeNew_min)/(rangeOld_max - rangeOld_min);
}

double convert(double (*scalingFun)(double, double, double, double), double rangeOld_min, double rangeOld_max, double rangeNew_min, double rangeNew_max, double value)
{
    return (scalingFun(rangeOld_min, rangeOld_max, rangeNew_min, rangeNew_max) * (value - rangeOld_min)) + rangeNew_min;
}

double convert(double scalingFactor, double value, double rangeOld_min, double rangeNew_min)
{
    return scalingFactor*(value - rangeOld_min) + rangeNew_min;
}

double getVector2dLenght(double x, double y)
{
    return sqrt(pow(x,2) + pow(y,2));
}
