#pragma once
#include <cstdint>
#include "Vector3.h"

namespace Lite {

// Fast, deterministic pseudo-random number generator (XorShift32)
// Ideal for physics engines where reproducibility is critical.
class Random {
private:
    uint32_t state;

public:
    Random(uint32_t seed = 123456789) {
        state = seed;
        if (state == 0) state = 1; // State can never be 0
    }

    uint32_t nextInt() {
        uint32_t x = state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        state = x;
        return x;
    }

    // Returns a real between 0.0 and 1.0
    real nextReal() {
        return (real)nextInt() / (real)UINT32_MAX;
    }

    // Returns a real between min and max
    real range(real min, real max) {
        return min + nextReal() * (max - min);
    }

    // Returns a random direction vector on the unit sphere
    Vector3 randomDirection() {
        real z = range(-1.0f, 1.0f);
        real a = range(0.0f, 6.2831853f); // 2*PI
        real r = std::sqrt(1.0f - z * z);
        return Vector3(r * std::cos(a), r * std::sin(a), z);
    }
};

} // namespace Lite
