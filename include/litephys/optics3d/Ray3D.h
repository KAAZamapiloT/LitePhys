#pragma once
#include "../math/Vector3.h"

namespace Lite {

struct Ray3D {
    Vector3 origin;
    Vector3 direction;

    Ray3D(const Vector3& origin, const Vector3& direction)
        : origin(origin), direction(direction.normalized()) {}
};

} // namespace Lite
