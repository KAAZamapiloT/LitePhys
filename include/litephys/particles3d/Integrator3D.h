#pragma once
#include "PhysicsState3D.h"

namespace Lite {

class Integrator3D {
public:
    static void integrate(PhysicsState3D& state, real dt) {
        if (state.inverseMass <= 0.0f) return;

        state.position += state.velocity * dt;

        Vector3 acceleration = state.force * state.inverseMass;
        state.velocity += acceleration * dt;

        state.velocity *= Math::real_pow(state.damping, dt);

        state.force.clear();
    }
};

} // namespace Lite
