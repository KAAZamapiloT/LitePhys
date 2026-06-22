#pragma once
#include "PhysicsState3D.h"
#include <vector>
#include <execution>
#include <algorithm>

namespace Lite {

class ParallelIntegrator3D {
public:
    // Integrates an entire array of particles concurrently using C++ execution policies
    static void integrate(std::vector<PhysicsState3D>& states, real dt) {
        std::for_each(std::execution::par_unseq, states.begin(), states.end(), [dt](PhysicsState3D& state) {
            if (state.inverseMass <= 0.0f) return;
            
            // Linear integration
            Vector3 acceleration = state.force * state.inverseMass;
            state.velocity = state.velocity + acceleration * dt;
            state.position = state.position + state.velocity * dt;

            // Clear forces
            state.force = Vector3();
        });
    }
    
    // Allows applying a global gravity force concurrently
    static void applyGravity(std::vector<PhysicsState3D>& states, const Vector3& gravity) {
        std::for_each(std::execution::par_unseq, states.begin(), states.end(), [&](PhysicsState3D& state) {
            if (state.inverseMass > 0.0f) {
                state.force += gravity * state.mass;
            }
        });
    }
};

} // namespace Lite
