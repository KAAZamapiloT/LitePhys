#pragma once
#include "../math/Vector3.h"

namespace Lite {

struct CollisionManifold3D {
    Vector3 contactPoint;
    Vector3 contactNormal;
    real penetration;
    
    // For Warm Starting & Friction
    real accumulatedNormalImpulse = 0.0f;
    Vector3 accumulatedTangentImpulse = Vector3(0,0,0);
};

} // namespace Lite
