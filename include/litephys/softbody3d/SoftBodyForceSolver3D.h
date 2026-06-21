#pragma once
#include "SoftBodyState3D.h"
#include "../particles3d/ForceSolver3D.h"

namespace Lite {

class SoftBodyForceSolver3D {
public:
    static void applyGravity(SoftBodyState3D& softbody, const Vector3& gravity) {
        for (auto& particle : softbody.particles) {
            ForceSolver3D::applyGravity(particle, gravity);
        }
    }

    static void applySpringForces(SoftBodyState3D& softbody) {
        for (const auto& spring : softbody.springs) {
            PhysicsState3D& pA = softbody.particles[spring.indexA];
            PhysicsState3D& pB = softbody.particles[spring.indexB];

            Vector3 diff = pB.position - pA.position;
            real dist = diff.magnitude();
            if (dist < 0.0001f) continue;
            
            Vector3 dir = diff * (1.0f / dist);

            // Hooke's Law (Spring Force)
            real offset = dist - spring.restLength;
            real springForce = offset * spring.stiffness;

            // Damping Force
            Vector3 relVel = pB.velocity - pA.velocity;
            real dampForce = relVel.dot(dir) * spring.damping;

            Vector3 totalForce = dir * (springForce + dampForce);

            pA.force += totalForce;
            pB.force -= totalForce;
        }
    }
};

} // namespace Lite
