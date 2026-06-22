#pragma once
#include "../math/Vector4.h"

namespace Lite {

struct Ray4D {
    Vector4 origin;
    Vector4 direction; // Should be normalized

    Ray4D() {}
    Ray4D(const Vector4& o, const Vector4& d) : origin(o), direction(d) {}

    Vector4 getPoint(real distance) const {
        return origin + direction * distance;
    }
};

} // namespace Lite
