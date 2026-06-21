#pragma once
#include "PhysicsState4D.h"

namespace Lite {

class Integrator4D {
public:
    static void integrate(PhysicsState4D& state, real dt) {
        if (state.inverseMass <= 0.0f) return;

        state.position += state.velocity * dt;

        Vector4 acceleration = state.force * state.inverseMass;
        state.velocity += acceleration * dt;

        state.velocity *= Math::real_pow(state.damping, dt);

        state.force.clear();
    }
};

} // namespace Lite
