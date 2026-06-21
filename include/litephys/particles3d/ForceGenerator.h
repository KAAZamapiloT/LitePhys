#pragma once
#include "Particle.h"
#include <vector>

namespace Lite {

// Interface for objects that can apply force to a particle
class ParticleForceGenerator {
public:
    virtual ~ParticleForceGenerator() {}
    
    // Overload this to calculate and apply the force
    virtual void updateForce(Particle* particle, real duration) = 0;
};

// Holds all the force generators and the particles they apply to
class ParticleForceRegistry {
protected:
    struct ParticleForceRegistration {
        Particle* particle;
        ParticleForceGenerator* fg;
    };

    typedef std::vector<ParticleForceRegistration> Registry;
    Registry registrations;

public:
    void add(Particle* particle, ParticleForceGenerator* fg) {
        ParticleForceRegistration registration;
        registration.particle = particle;
        registration.fg = fg;
        registrations.push_back(registration);
    }

    void remove(Particle* particle, ParticleForceGenerator* fg) {
        for (auto it = registrations.begin(); it != registrations.end(); ++it) {
            if (it->particle == particle && it->fg == fg) {
                registrations.erase(it);
                break;
            }
        }
    }

    void clear() {
        registrations.clear();
    }

    void updateForces(real duration) {
        for (auto& i : registrations) {
            i.fg->updateForce(i.particle, duration);
        }
    }
};

} // namespace Lite
