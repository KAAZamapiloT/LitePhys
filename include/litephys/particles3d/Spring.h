#pragma once
#include "ForceGenerator.h"

namespace Lite {

class ParticleSpring : public ParticleForceGenerator {
    // The particle at the other end of the spring
    Particle* other;

    // Spring constant
    real springConstant;

    // Rest length of the spring
    real restLength;

public:
    ParticleSpring(Particle* other, real springConstant, real restLength)
        : other(other), springConstant(springConstant), restLength(restLength) {}

    virtual void updateForce(Particle* particle, real duration) override {
        // Calculate the vector of the spring
        Vector3 force = particle->getPosition();
        force -= other->getPosition();

        // Calculate the magnitude of the force
        real magnitude = force.magnitude();
        
        if (magnitude <= 0) return; // Prevent division by zero and zero distance

        magnitude = Math::real_abs(magnitude - restLength);
        magnitude *= springConstant;

        // Calculate the final force and apply it
        force.normalize();
        force *= -magnitude;
        particle->addForce(force);
    }
};

} // namespace Lite
