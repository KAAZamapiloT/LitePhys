#pragma once
#include <vector>
#include "../particles3d/PhysicsState3D.h"

namespace Lite {

struct SpringConstraint3D {
    int indexA;
    int indexB;
    real restLength;
    real stiffness;
    real damping;
    
    SpringConstraint3D(int a, int b, real r, real s, real d = 0.05f) 
        : indexA(a), indexB(b), restLength(r), stiffness(s), damping(d) {}
};

struct SoftBodyState3D {
    std::vector<PhysicsState3D> particles;
    std::vector<SpringConstraint3D> springs;
};

} // namespace Lite
