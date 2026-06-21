#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <litephys/LitePhys3D.h>
#include <litephys/LiteCollisions3D.h>

using namespace Lite;

SpatialHash3D spatialHash(2.0f); 
CollisionResolver3D collisionResolver;
std::vector<CollidableParticle*> collidableParticles;

const real BOX_SIZE = 10.0f;

void checkBounds(Particle* baseParticle, real duration) {
    CollidableParticle* p = static_cast<CollidableParticle*>(baseParticle);
    Vector3 pos = p->getPosition();
    Vector3 vel = p->getVelocity();
    real r = p->radius;

    if (pos.y < -BOX_SIZE + r) { pos.y = -BOX_SIZE + r; vel.y *= -p->restitution; }
    if (pos.y >  BOX_SIZE - r) { pos.y =  BOX_SIZE - r; vel.y *= -p->restitution; }
    if (pos.x < -BOX_SIZE + r) { pos.x = -BOX_SIZE + r; vel.x *= -p->restitution; }
    if (pos.x >  BOX_SIZE - r) { pos.x =  BOX_SIZE - r; vel.x *= -p->restitution; }
    if (pos.z < -BOX_SIZE + r) { pos.z = -BOX_SIZE + r; vel.z *= -p->restitution; }
    if (pos.z >  BOX_SIZE - r) { pos.z =  BOX_SIZE - r; vel.z *= -p->restitution; }

    p->setPosition(pos);
    p->setVelocity(vel);
}

int main() {
    std::srand((unsigned int)std::time(nullptr));
    std::cout << "LitePhys - Fast 3D Collisions Example" << std::endl;

    ParticleWorld world;
    ParticleGravity gravity(Vector3(0, -9.81f, 0));
    
    collisionResolver.mode = CollisionMode::FAST_IMPULSE;

    for (int i = 0; i < 200; ++i) {
        real x = ((std::rand() % 1600) / 100.0f) - 8.0f;
        real y = ((std::rand() % 1600) / 100.0f) - 8.0f;
        real z = ((std::rand() % 1600) / 100.0f) - 8.0f;

        CollidableParticle* p = new CollidableParticle(1.0f, Vector3(x, y, z), 1.0f, 0.8f);
        p->setVelocity(Vector3(
            ((std::rand() % 1000) / 100.0f) - 5.0f,
            ((std::rand() % 1000) / 100.0f) - 5.0f,
            ((std::rand() % 1000) / 100.0f) - 5.0f
        ));

        world.addParticle(p);
        world.getForceRegistry().add(p, &gravity);
        collidableParticles.push_back(p);
    }

    world.setPostUpdateHook(checkBounds);

    std::cout << "Simulating 100 frames with 200 colliding particles..." << std::endl;
    for (int frame = 0; frame < 100; ++frame) {
        world.startFrame();
        world.runPhysics(0.016f);
        collisionResolver.resolve(collidableParticles, spatialHash);
    }

    std::cout << "Simulation completed incredibly fast thanks to Spatial Hashing!" << std::endl;

    for (auto* p : collidableParticles) delete p;
    return 0;
}
