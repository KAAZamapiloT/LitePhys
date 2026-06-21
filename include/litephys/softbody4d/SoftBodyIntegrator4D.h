#pragma once
#include "SoftBodyState4D.h"
#include "../particles4d/Integrator4D.h"

namespace Lite {

class SoftBodyIntegrator4D {
public:
    static void integrate(SoftBodyState4D& softbody, real dt) {
        for (auto& particle : softbody.particles) {
            Integrator4D::integrate(particle, dt);
        }
    }
};

} // namespace Lite
