#pragma once
#include "SoftBodyState3D.h"
#include "../particles3d/Integrator3D.h"

namespace Lite {

class SoftBodyIntegrator3D {
public:
    static void integrate(SoftBodyState3D& softbody, real dt) {
        for (auto& particle : softbody.particles) {
            Integrator3D::integrate(particle, dt);
        }
    }
};

} // namespace Lite
