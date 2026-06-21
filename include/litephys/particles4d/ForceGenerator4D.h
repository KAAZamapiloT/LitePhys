#pragma once
#include "Particle4D.h"
#include <vector>

namespace Lite {

class ParticleForceGenerator4D {
public:
    virtual ~ParticleForceGenerator4D() {}
    virtual void updateForce(Particle4D* particle, real duration) = 0;
};

class ParticleForceRegistry4D {
protected:
    struct ParticleForceRegistration {
        Particle4D* particle;
        ParticleForceGenerator4D* fg;
    };
    typedef std::vector<ParticleForceRegistration> Registry;
    Registry registrations;

public:
    void add(Particle4D* particle, ParticleForceGenerator4D* fg) {
        ParticleForceRegistration registration;
        registration.particle = particle;
        registration.fg = fg;
        registrations.push_back(registration);
    }
    void remove(Particle4D* particle, ParticleForceGenerator4D* fg) {
        for (auto it = registrations.begin(); it != registrations.end(); ++it) {
            if (it->particle == particle && it->fg == fg) {
                registrations.erase(it);
                break;
            }
        }
    }
    void clear() { registrations.clear(); }
    void updateForces(real duration) {
        for (auto& i : registrations) {
            i.fg->updateForce(i.particle, duration);
        }
    }
};

} // namespace Lite
