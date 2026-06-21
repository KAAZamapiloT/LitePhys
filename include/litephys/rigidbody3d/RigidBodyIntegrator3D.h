#pragma once
#include "RigidBodyState3D.h"

namespace Lite {

class RigidBodyIntegrator3D {
public:
    static void integrate(RigidBodyState3D& state, real dt) {
        if (state.inverseMass <= 0.0f) return; // Infinite mass / static body

        // 1. Linear integration
        Vector3 linearAcceleration = state.force * state.inverseMass;
        state.velocity = state.velocity + linearAcceleration * dt;
        state.position = state.position + state.velocity * dt;

        // 2. Angular integration
        Vector3 angularAcceleration = state.inverseInertiaTensorWorld * state.torque;
        state.angularVelocity = state.angularVelocity + angularAcceleration * dt;
        
        // Update orientation quaternion using angular velocity
        state.orientation.addScaledVector(state.angularVelocity, dt);
        state.orientation.normalize();

        // 3. Update world inertia tensor based on new orientation
        calculateDerivedData(state);

        // 4. Clear accumulators
        state.force = Vector3();
        state.torque = Vector3();
    }

    static void calculateDerivedData(RigidBodyState3D& state) {
        // Calculate the transform matrix from the orientation
        Matrix3 rotMat = state.orientation.toRotationMatrix();
        
        // Inverse inertia tensor in world space: I^-1_world = R * I^-1_local * R^T
        state.inverseInertiaTensorWorld = rotMat * state.inverseInertiaTensor * rotMat.transpose();
    }
};

} // namespace Lite
