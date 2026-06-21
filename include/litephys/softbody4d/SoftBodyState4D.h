#pragma once
#include <vector>
#include "../particles4d/PhysicsState4D.h"

namespace Lite {

struct SpringConstraint4D {
    int indexA;
    int indexB;
    real restLength;
    real stiffness;
    real damping;
    
    SpringConstraint4D(int a, int b, real r, real s, real d = 0.05f) 
        : indexA(a), indexB(b), restLength(r), stiffness(s), damping(d) {}
};

struct SoftBodyState4D {
    std::vector<PhysicsState4D> particles;
    std::vector<SpringConstraint4D> springs;
};

} // namespace Lite
