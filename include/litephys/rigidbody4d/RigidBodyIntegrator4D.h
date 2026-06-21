#pragma once
#include "RigidBodyState4D.h"

namespace Lite {

class RigidBodyIntegrator4D {
public:
    static void integrate(RigidBodyState4D& state, real dt) {
        if (state.inverseMass <= 0.0f) return; // Infinite mass / static body

        // 1. Linear integration
        Vector4 linearAcceleration = state.force * state.inverseMass;
        state.velocity = state.velocity + linearAcceleration * dt;
        state.position = state.position + state.velocity * dt;

        // 2. Angular integration
        Bivector4 angularAcceleration = state.inverseInertiaTensorWorld * state.torque;
        state.angularVelocity = state.angularVelocity + angularAcceleration * dt;
        
        // Update orientation matrix using angular velocity
        Matrix4 W = state.angularVelocity.toSkewSymmetricMatrix();
        state.orientation = state.orientation + (W * state.orientation) * dt;
        state.orientation.orthogonalize();

        // 3. Update world inertia tensor based on new orientation
        calculateDerivedData(state);

        // 4. Clear accumulators
        state.force = Vector4();
        state.torque = Bivector4();
    }

    static void calculateDerivedData(RigidBodyState4D& state) {
        // Inverse inertia tensor in world space
        // For hyper-spheres this is completely symmetric and orientation invariant, so we don't need to do complex tensor rotation.
        // For arbitrary 4D shapes, rotating a 6x6 rank-4 tensor is computationally heavy.
        // As a fast library, we will assume isotropic inertia or pre-baked tensor for spheres in Phase 1.
        state.inverseInertiaTensorWorld = state.inverseInertiaTensor;
    }
};

} // namespace Lite
