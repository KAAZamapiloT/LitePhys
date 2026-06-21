#pragma once
#include "RigidBodyState4D.h"

namespace Lite {

class RigidBodyForceSolver4D {
public:
    static void applyGravity(RigidBodyState4D& state, const Vector4& gravity) {
        if (state.inverseMass > 0.0f) {
            state.force = state.force + gravity * state.mass;
        }
    }

    static void applyForceCenter(RigidBodyState4D& state, const Vector4& force) {
        state.force = state.force + force;
    }

    // Applies a force at a specific world point (generates torque if off-center)
    static void applyForceAtPoint(RigidBodyState4D& state, const Vector4& force, const Vector4& point) {
        // point is in world space
        Vector4 pt = point - state.position;
        state.force = state.force + force;
        
        // Torque is the wedge product of radius ^ force
        state.torque = state.torque + Bivector4::wedge(pt, force);
    }
};

} // namespace Lite
