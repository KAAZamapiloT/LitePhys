#pragma once
#include "RigidBodyState3D.h"

namespace Lite {

class RigidBodyForceSolver3D {
public:
    static void applyGravity(RigidBodyState3D& state, const Vector3& gravity) {
        if (state.inverseMass > 0.0f) {
            state.force = state.force + gravity * state.mass;
        }
    }

    static void applyForceCenter(RigidBodyState3D& state, const Vector3& force) {
        state.force = state.force + force;
    }

    // Applies a force at a specific world point (generates torque if off-center)
    static void applyForceAtPoint(RigidBodyState3D& state, const Vector3& force, const Vector3& point) {
        // point is in world space
        Vector3 pt = point - state.position;
        state.force = state.force + force;
        state.torque = state.torque + pt.cross(force);
    }
};

} // namespace Lite
