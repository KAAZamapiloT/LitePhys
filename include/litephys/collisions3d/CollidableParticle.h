#pragma once
#include "../particles3d/Particle.h"

namespace Lite {

class CollidableParticle : public Particle {
public:
    real radius;
    real restitution;

    CollidableParticle() : Particle(), radius(1.0f), restitution(0.5f) {}
    
    CollidableParticle(real mass, const Vector3& pos, real radius = 1.0f, real restitution = 0.5f)
        : Particle(), radius(radius), restitution(restitution) {
        setMass(mass);
        setPosition(pos);
    }
};

} // namespace Lite
