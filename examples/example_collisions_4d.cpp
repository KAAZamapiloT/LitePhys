#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <litephys/LitePhys4D.h>
#include <litephys/LiteCollisions4D.h>

using namespace Lite;

SpatialHash4D spatialHash4D(2.0f);
CollisionResolver4D collisionResolver4D;
std::vector<CollidableParticle4D*> collidableParticles4D;

const real BOX_SIZE = 10.0f;

void checkBounds4D(Particle4D* baseParticle, real duration) {
    CollidableParticle4D* p = static_cast<CollidableParticle4D*>(baseParticle);
    Vector4 pos = p->getPosition();
    Vector4 vel = p->getVelocity();
    real r = p->radius;

    if (pos.y < -BOX_SIZE + r) { pos.y = -BOX_SIZE + r; vel.y *= -p->restitution; }
    if (pos.y >  BOX_SIZE - r) { pos.y =  BOX_SIZE - r; vel.y *= -p->restitution; }
    if (pos.x < -BOX_SIZE + r) { pos.x = -BOX_SIZE + r; vel.x *= -p->restitution; }
    if (pos.x >  BOX_SIZE - r) { pos.x =  BOX_SIZE - r; vel.x *= -p->restitution; }
    if (pos.z < -BOX_SIZE + r) { pos.z = -BOX_SIZE + r; vel.z *= -p->restitution; }
    if (pos.z >  BOX_SIZE - r) { pos.z =  BOX_SIZE - r; vel.z *= -p->restitution; }
    if (pos.w < -BOX_SIZE + r) { pos.w = -BOX_SIZE + r; vel.w *= -p->restitution; }
    if (pos.w >  BOX_SIZE - r) { pos.w =  BOX_SIZE - r; vel.w *= -p->restitution; }

    p->setPosition(pos);
    p->setVelocity(vel);
}

int main() {
    std::srand((unsigned int)std::time(nullptr));
    std::cout << "LitePhys - Fast 4D Collisions Example" << std::endl;

    ParticleWorld4D world;
    ParticleGravity4D gravity(Vector4(0, -9.81f, 0, -5.0f));
    
    collisionResolver4D.mode = CollisionMode::FAST_IMPULSE;

    for (int i = 0; i < 100; ++i) {
        real x = ((std::rand() % 1600) / 100.0f) - 8.0f;
        real y = ((std::rand() % 1600) / 100.0f) - 8.0f;
        real z = ((std::rand() % 1600) / 100.0f) - 8.0f;
        real w = ((std::rand() % 1600) / 100.0f) - 8.0f;

        CollidableParticle4D* p = new CollidableParticle4D(1.0f, Vector4(x, y, z, w), 1.0f, 0.8f);
        p->setVelocity(Vector4(
            ((std::rand() % 1000) / 100.0f) - 5.0f,
            ((std::rand() % 1000) / 100.0f) - 5.0f,
            ((std::rand() % 1000) / 100.0f) - 5.0f,
            ((std::rand() % 1000) / 100.0f) - 5.0f
        ));

        world.addParticle(p);
        world.getForceRegistry().add(p, &gravity);
        collidableParticles4D.push_back(p);
    }

    world.setPostUpdateHook(checkBounds4D);

    std::cout << "Simulating 100 frames with 100 colliding 4D particles..." << std::endl;
    for (int frame = 0; frame < 100; ++frame) {
        world.startFrame();
        world.runPhysics(0.016f);
        collisionResolver4D.resolve(collidableParticles4D, spatialHash4D);
    }

    std::cout << "4D Simulation completed incredibly fast!" << std::endl;

    for (auto* p : collidableParticles4D) delete p;
    return 0;
}
