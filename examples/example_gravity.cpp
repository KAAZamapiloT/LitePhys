#include <iostream>
#include <iomanip>
#include <litephys/LitePhys3D.h>

using namespace Lite;

// A simple hook to demonstrate user tweakability (like resolving collisions with a floor)
// This is exactly the kind of "tweakable function" a developer would add when hooking into their engine
void checkBounds(Particle* p, real duration) {
    if (p->getPosition().y < 0.0f) {
        // Simple bounce on the ground
        Vector3 pos = p->getPosition();
        pos.y = 0.0f;
        p->setPosition(pos);

        Vector3 vel = p->getVelocity();
        // Reverse velocity and apply a simple restitution (bounciness) factor
        vel.y = -vel.y * 0.8f; 
        p->setVelocity(vel);
    }
}

int main() {
    std::cout << "LitePhys - Gravity Example" << std::endl;

    ParticleWorld world;
    
    // Add custom developer hook to handle bounds/collisions without bloating the physics core
    world.setPostUpdateHook(checkBounds);

    Particle p1;
    p1.setPosition(0, 10, 0); // Start 10 units high
    p1.setMass(2.0f);
    
    world.addParticle(&p1);

    ParticleGravity gravity(Vector3(0, -9.81f, 0));
    world.getForceRegistry().add(&p1, &gravity);

    // Simulate for 2 seconds with a fixed time step
    const real dt = 0.1f;
    for (int i = 0; i <= 20; ++i) {
        std::cout << "Time: " << std::fixed << std::setprecision(1) << (i * dt) << "s - ";
        std::cout << "Position: (" 
                  << std::setprecision(2) << p1.getPosition().x << ", " 
                  << p1.getPosition().y << ", " 
                  << p1.getPosition().z << ")" << std::endl;
        
        world.startFrame();
        world.runPhysics(dt);
    }

    return 0;
}
