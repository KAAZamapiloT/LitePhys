#pragma once
#include "SoftBodyState4D.h"
#include "../particles4d/ForceSolver4D.h"

namespace Lite {

class SoftBodyForceSolver4D {
public:
    static void applyGravity(SoftBodyState4D& softbody, const Vector4& gravity) {
        for (auto& particle : softbody.particles) {
            ForceSolver4D::applyGravity(particle, gravity);
        }
    }

    static void applySpringForces(SoftBodyState4D& softbody) {
        for (const auto& spring : softbody.springs) {
            PhysicsState4D& pA = softbody.particles[spring.indexA];
            PhysicsState4D& pB = softbody.particles[spring.indexB];

            Vector4 diff = pB.position - pA.position;
            real dist = diff.magnitude();
            if (dist < 0.0001f) continue;
            
            Vector4 dir = diff * (1.0f / dist);

            // Hooke's Law (Spring Force)
            real offset = dist - spring.restLength;
            real springForce = offset * spring.stiffness;

            // Damping Force
            Vector4 relVel = pB.velocity - pA.velocity;
            real dampForce = relVel.dot(dir) * spring.damping;

            Vector4 totalForce = dir * (springForce + dampForce);

            pA.force += totalForce;
            pB.force -= totalForce;
        }
    }
};

} // namespace Lite
