#pragma once
#include "../math/Vector3.h"

namespace Lite {

struct Ray3D {
    Vector3 origin;
    Vector3 direction; // Should be normalized

    Ray3D() {}
    Ray3D(const Vector3& o, const Vector3& d) : origin(o), direction(d) {}

    Vector3 getPoint(real distance) const {
        return origin + direction * distance;
    }
};

} // namespace Lite
