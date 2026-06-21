#pragma once
#include "../particles4d/Particle4D.h"

namespace Lite {

class CollidableParticle4D : public Particle4D {
public:
    real radius;
    real restitution;

    CollidableParticle4D() : Particle4D(), radius(1.0f), restitution(0.5f) {}
    
    CollidableParticle4D(real mass, const Vector4& pos, real radius = 1.0f, real restitution = 0.5f)
        : Particle4D(), radius(radius), restitution(restitution) {
        setMass(mass);
        setPosition(pos);
    }
};

} // namespace Lite
