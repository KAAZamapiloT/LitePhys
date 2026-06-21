#pragma once
#include "ForceGenerator4D.h"

namespace Lite {

class ParticleSpring4D : public ParticleForceGenerator4D {
    Particle4D* other;
    real springConstant;
    real restLength;
public:
    ParticleSpring4D(Particle4D* other, real springConstant, real restLength)
        : other(other), springConstant(springConstant), restLength(restLength) {}

    virtual void updateForce(Particle4D* particle, real duration) override {
        Vector4 force = particle->getPosition();
        force -= other->getPosition();

        real magnitude = force.magnitude();
        if (magnitude <= 0) return;

        magnitude = Math::real_abs(magnitude - restLength);
        magnitude *= springConstant;

        force.normalize();
        force *= -magnitude;
        particle->addForce(force);
    }
};

} // namespace Lite
