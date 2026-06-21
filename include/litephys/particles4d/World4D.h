#pragma once
#include "Particle4D.h"
#include "ForceGenerator4D.h"
#include <vector>

namespace Lite {

class ParticleWorld4D {
    typedef std::vector<Particle4D*> Particles;
    Particles particles;
    
    ParticleForceRegistry4D registry;
    void (*postUpdateHook)(Particle4D*, real);

public:
    ParticleWorld4D() : postUpdateHook(nullptr) {}

    void addParticle(Particle4D* p) { particles.push_back(p); }
    void removeParticle(Particle4D* p) {
        for (auto it = particles.begin(); it != particles.end(); ++it) {
            if (*it == p) {
                particles.erase(it);
                break;
            }
        }
    }

    ParticleForceRegistry4D& getForceRegistry() { return registry; }

    void setPostUpdateHook(void (*hook)(Particle4D*, real)) { postUpdateHook = hook; }

    void startFrame() {
        for (auto& p : particles) p->clearAccumulator();
    }

    void runPhysics(real duration) {
        registry.updateForces(duration);
        for (auto& p : particles) {
            p->integrate(duration);
            if (postUpdateHook) postUpdateHook(p, duration);
        }
    }
};

} // namespace Lite
