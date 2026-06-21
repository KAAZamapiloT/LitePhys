#pragma once
#include "../math/Vector4.h"

namespace Lite {

struct Ray4D {
    Vector4 origin;
    Vector4 direction;

    Ray4D(const Vector4& origin, const Vector4& direction)
        : origin(origin), direction(direction.normalized()) {}
};

} // namespace Lite
