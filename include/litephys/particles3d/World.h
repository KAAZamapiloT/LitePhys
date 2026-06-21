#pragma once
#include "Particle.h"
#include "ForceGenerator.h"
#include <vector>

namespace Lite {

class ParticleWorld {
    typedef std::vector<Particle*> Particles;
    Particles particles;
    
    ParticleForceRegistry registry;
    
    // User-defined callback that can be tweaked during updates
    // This allows custom engine hooks without modifying the library
    void (*postUpdateHook)(Particle*, real);

public:
    ParticleWorld() : postUpdateHook(nullptr) {}

    void addParticle(Particle* p) {
        particles.push_back(p);
    }

    void removeParticle(Particle* p) {
        for (auto it = particles.begin(); it != particles.end(); ++it) {
            if (*it == p) {
                particles.erase(it);
                break;
            }
        }
    }

    ParticleForceRegistry& getForceRegistry() {
        return registry;
    }

    // Set a hook to be called after integration
    // Developers can use this to add collision constraints, bounds checking, etc.
    void setPostUpdateHook(void (*hook)(Particle*, real)) {
        postUpdateHook = hook;
    }

    // Advance the physics simulation
    void startFrame() {
        for (auto& p : particles) {
            p->clearAccumulator();
        }
    }

    void runPhysics(real duration) {
        // Apply forces
        registry.updateForces(duration);

        // Integrate
        for (auto& p : particles) {
            p->integrate(duration);
            
            if (postUpdateHook) {
                postUpdateHook(p, duration);
            }
        }
    }
};

} // namespace Lite
