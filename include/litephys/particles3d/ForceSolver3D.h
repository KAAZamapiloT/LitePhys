#pragma once
#include "PhysicsState3D.h"

namespace Lite {

class ForceSolver3D {
public:
    static void applyGravity(PhysicsState3D& state, const Vector3& gravity) {
        if (state.inverseMass <= 0.0f) return;
        state.force += gravity * state.mass;
    }

    static void applySpring(PhysicsState3D& stateA, PhysicsState3D& stateB, real restLength, real stiffness) {
        Vector3 diff = stateA.position - stateB.position;
        real dist = diff.magnitude();
        if (dist < 0.0001f) return;
        
        real magnitude = dist - restLength;
        magnitude *= stiffness;
        
        Vector3 force = diff.normalized() * -magnitude;
        stateA.force += force;
        stateB.force -= force;
    }
};

} // namespace Lite
