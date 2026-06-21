#pragma once
#include "../math/Vector4.h"

namespace Lite {

struct CollisionManifold4D {
    Vector4 contactPoint;
    Vector4 contactNormal;
    real penetration;
};

} // namespace Lite
