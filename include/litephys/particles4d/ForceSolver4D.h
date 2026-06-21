#pragma once
#include "PhysicsState4D.h"

namespace Lite {

class ForceSolver4D {
public:
    static void applyGravity(PhysicsState4D& state, const Vector4& gravity) {
        if (state.inverseMass <= 0.0f) return;
        state.force += gravity * state.mass;
    }

    static void applySpring(PhysicsState4D& stateA, PhysicsState4D& stateB, real restLength, real stiffness) {
        Vector4 diff = stateA.position - stateB.position;
        real dist = diff.magnitude();
        if (dist < 0.0001f) return;
        
        real magnitude = dist - restLength;
        magnitude *= stiffness;
        
        Vector4 force = diff.normalized() * -magnitude;
        stateA.force += force;
        stateB.force -= force;
    }
};

} // namespace Lite
