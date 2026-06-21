#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <litephys/LitePhys4D.h>
#include <litephys/LiteCollisions4D.h>

using namespace Lite;

const real BOX_SIZE = 10.0f;

int main() {
    std::srand((unsigned int)std::time(nullptr));
    std::cout << "LitePhys - Stateless ECS 4D Collisions Example" << std::endl;

    std::vector<PhysicsState4D> entities(100);

    for (int i = 0; i < 100; ++i) {
        real x = ((std::rand() % 1600) / 100.0f) - 8.0f;
        real y = ((std::rand() % 1600) / 100.0f) - 8.0f;
        real z = ((std::rand() % 1600) / 100.0f) - 8.0f;
        real w = ((std::rand() % 1600) / 100.0f) - 8.0f;

        entities[i].position = Vector4(x, y, z, w);
        entities[i].velocity = Vector4(
            ((std::rand() % 1000) / 100.0f) - 5.0f,
            ((std::rand() % 1000) / 100.0f) - 5.0f,
            ((std::rand() % 1000) / 100.0f) - 5.0f,
            ((std::rand() % 1000) / 100.0f) - 5.0f
        );
        entities[i].radius = 1.0f;
        entities[i].restitution = 0.8f;
        entities[i].setMass(1.0f);
    }

    SpatialHash4D spatialHash(2.0f);
    Vector4 gravity(0, -9.81f, 0, -5.0f);
    const real dt = 0.016f;

    std::cout << "Simulating 100 frames with 100 colliding 4D components..." << std::endl;
    for (int frame = 0; frame < 100; ++frame) {
        
        for (auto& state : entities) {
            ForceSolver4D::applyGravity(state, gravity);
            Integrator4D::integrate(state, dt);
        }

        CollisionResolver4D::resolve(entities, spatialHash, CollisionMode::FAST_IMPULSE);

        for (auto& state : entities) {
            real r = state.radius;
            if (state.position.y < -BOX_SIZE + r) { state.position.y = -BOX_SIZE + r; state.velocity.y *= -state.restitution; }
            if (state.position.y >  BOX_SIZE - r) { state.position.y =  BOX_SIZE - r; state.velocity.y *= -state.restitution; }
            if (state.position.x < -BOX_SIZE + r) { state.position.x = -BOX_SIZE + r; state.velocity.x *= -state.restitution; }
            if (state.position.x >  BOX_SIZE - r) { state.position.x =  BOX_SIZE - r; state.velocity.x *= -state.restitution; }
            if (state.position.z < -BOX_SIZE + r) { state.position.z = -BOX_SIZE + r; state.velocity.z *= -state.restitution; }
            if (state.position.z >  BOX_SIZE - r) { state.position.z =  BOX_SIZE - r; state.velocity.z *= -state.restitution; }
            if (state.position.w < -BOX_SIZE + r) { state.position.w = -BOX_SIZE + r; state.velocity.w *= -state.restitution; }
            if (state.position.w >  BOX_SIZE - r) { state.position.w =  BOX_SIZE - r; state.velocity.w *= -state.restitution; }
        }
    }

    std::cout << "4D Simulation completed purely functionally!" << std::endl;
    return 0;
}
