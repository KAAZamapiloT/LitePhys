#include <iostream>
#include <iomanip>
#include <litephys/LitePhys4D.h>

using namespace Lite;

// Tweakable hook to demonstrate bouncing on a 3D hyperplane within 4D space
void checkBounds4D(Particle4D* p, real duration) {
    // Standard 3D floor bounce (y=0)
    if (p->getPosition().y < 0.0f) {
        Vector4 pos = p->getPosition();
        pos.y = 0.0f;
        p->setPosition(pos);

        Vector4 vel = p->getVelocity();
        vel.y = -vel.y * 0.8f; 
        p->setVelocity(vel);
    }

    // 4D boundary bounce (w=0)
    if (p->getPosition().w < 0.0f) {
        Vector4 pos = p->getPosition();
        pos.w = 0.0f;
        p->setPosition(pos);

        Vector4 vel = p->getVelocity();
        vel.w = -vel.w * 0.8f; 
        p->setVelocity(vel);
    }
}

int main() {
    std::cout << "LitePhys - 4D Gravity & Rotation Example" << std::endl;

    ParticleWorld4D world;
    world.setPostUpdateHook(checkBounds4D);

    Particle4D p1;
    // Start at y=10 and w=5
    p1.setPosition(0, 10, 0, 5); 
    p1.setMass(2.0f);
    
    world.addParticle(&p1);

    // Apply gravity in both Y and W axes
    ParticleGravity4D gravity(Vector4(0, -9.81f, 0, -4.0f));
    world.getForceRegistry().add(&p1, &gravity);

    // Demonstrate 4D rotation via Matrix4
    // We will rotate the initial velocity slightly into the XW plane
    Matrix4 rotXW = Matrix4::makeRotationXW(Lite::PI / 4.0f); // 45 degrees
    Vector4 initialVelocity(2.0f, 0.0f, 0.0f, 0.0f);
    p1.setVelocity(rotXW.transform(initialVelocity));

    const real dt = 0.1f;
    for (int i = 0; i <= 20; ++i) {
        std::cout << "Time: " << std::fixed << std::setprecision(1) << (i * dt) << "s - ";
        std::cout << "Position: (" 
                  << std::setprecision(2) << p1.getPosition().x << ", " 
                  << p1.getPosition().y << ", " 
                  << p1.getPosition().z << ", "
                  << p1.getPosition().w << ")" << std::endl;
        
        world.startFrame();
        world.runPhysics(dt);
    }

    return 0;
}
