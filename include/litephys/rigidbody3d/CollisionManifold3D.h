#pragma once
#include "../math/Vector3.h"

namespace Lite {

struct CollisionManifold3D {
    Vector3 contactPoint;
    Vector3 contactNormal;
    real penetration;
};

} // namespace Lite
