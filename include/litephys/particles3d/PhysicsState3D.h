#pragma once
#include "../math/Vector3.h"
#include <limits>

namespace Lite {

struct PhysicsState3D {
    Vector3 position;
    Vector3 velocity;
    Vector3 force;
    
    real mass;
    real inverseMass;
    real damping;
    
    real radius;
    real restitution;

    PhysicsState3D() 
        : position(), velocity(), force(), 
          mass(1.0f), inverseMass(1.0f), damping(0.999f), 
          radius(1.0f), restitution(0.5f) {}

    void setMass(real m) {
        mass = m;
        if (m == 0.0f) {
            inverseMass = 0.0f;
        } else {
            inverseMass = 1.0f / m;
        }
    }
};

} // namespace Lite
