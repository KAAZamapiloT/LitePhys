#pragma once

#include <cmath>
#include <limits>

namespace Lite {

// Define the precision of the library
// Use #define LITE_PHYSICS_DOUBLE_PRECISION to use long double
// Use #define LITE_PHYSICS_APPROX_MATH to use fast approximations

#ifdef LITE_PHYSICS_DOUBLE_PRECISION
    typedef long double real;
#else
    typedef float real;
#endif

// Math constants
const real PI = static_cast<real>(3.14159265358979323846);

// Math utilities
namespace Math {
    
    inline real real_sqrt(real value) {
        return std::sqrt(value);
    }
    
    inline real real_abs(real value) {
        return std::abs(value);
    }

    inline real real_sin(real value) {
        return std::sin(value);
    }

    inline real real_cos(real value) {
        return std::cos(value);
    }

    inline real real_tan(real value) {
        return std::tan(value);
    }

    inline real real_pow(real base, real exponent) {
        return std::pow(base, exponent);
    }

    // Fast inverse square root (approximate mode)
    inline real fast_inv_sqrt(real number) {
#ifdef LITE_PHYSICS_APPROX_MATH
        // Quake III fast inverse square root for floats
        float x2 = static_cast<float>(number) * 0.5f;
        float y = static_cast<float>(number);
        long i = *(long*)&y;
        i = 0x5f3759df - (i >> 1);
        y = *(float*)&i;
        y = y * (1.5f - (x2 * y * y));
        return static_cast<real>(y);
#else
        return static_cast<real>(1.0) / std::sqrt(number);
#endif
    }
}

} // namespace Lite
