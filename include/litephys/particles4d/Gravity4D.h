#pragma once
#include "ForceGenerator4D.h"

namespace Lite {

class ParticleGravity4D : public ParticleForceGenerator4D {
    Vector4 gravity;
public:
    ParticleGravity4D(const Vector4& gravity) : gravity(gravity) {}
    virtual void updateForce(Particle4D* particle, real duration) override {
        if (!particle->hasFiniteMass()) return;
        particle->addForce(gravity * particle->getMass());
    }
};

} // namespace Lite
