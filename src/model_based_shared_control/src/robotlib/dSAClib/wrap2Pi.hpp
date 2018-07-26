#ifndef WRAP2PI_HPP
#define WRAP2PI_HPP
#include <math.h>

double wrap2Pi( double x ) {
    x = fmod(x+M_PI, 2.0*M_PI);
    if (x < 0) {
        x += 2.0 * M_PI;
    }
    return x-M_PI;
}

#endif
