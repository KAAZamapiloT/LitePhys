#pragma once
#include "ForceGenerator.h"

namespace Lite {

class ParticleGravity : public ParticleForceGenerator {
    Vector3 gravity;

public:
    ParticleGravity(const Vector3& gravity) : gravity(gravity) {}

    virtual void updateForce(Particle* particle, real duration) override {
        // Check that we do not have infinite mass
        if (!particle->hasFiniteMass()) return;

        // Apply the mass-scaled force to the particle
        particle->addForce(gravity * particle->getMass());
    }
};

} // namespace Lite
