#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <litephys/LitePhys3D.h>
#include <litephys/LiteCollisions3D.h>

using namespace Lite;

const real BOX_SIZE = 10.0f;

int main() {
    std::srand((unsigned int)std::time(nullptr));
    std::cout << "LitePhys - Stateless ECS 3D Collisions Example" << std::endl;

    // Game Engine owns the component array
    std::vector<PhysicsState3D> entities(200);

    for (int i = 0; i < 200; ++i) {
        real x = ((std::rand() % 1600) / 100.0f) - 8.0f;
        real y = ((std::rand() % 1600) / 100.0f) - 8.0f;
        real z = ((std::rand() % 1600) / 100.0f) - 8.0f;

        entities[i].position = Vector3(x, y, z);
        entities[i].velocity = Vector3(
            ((std::rand() % 1000) / 100.0f) - 5.0f,
            ((std::rand() % 1000) / 100.0f) - 5.0f,
            ((std::rand() % 1000) / 100.0f) - 5.0f
        );
        entities[i].radius = 1.0f;
        entities[i].restitution = 0.8f;
        entities[i].setMass(1.0f);
    }

    SpatialHash3D spatialHash(2.0f);
    Vector3 gravity(0, -9.81f, 0);
    const real dt = 0.016f;

    std::cout << "Simulating 100 frames with 200 colliding ECS components..." << std::endl;
    for (int frame = 0; frame < 100; ++frame) {
        
        // 1. Game engine applies forces and integrates
        for (auto& state : entities) {
            ForceSolver3D::applyGravity(state, gravity);
            Integrator3D::integrate(state, dt);
        }

        // 2. Game engine resolves internal collisions via the Physics Provider
        CollisionResolver3D::resolve(entities, spatialHash, CollisionMode::FAST_IMPULSE);

        // 3. Game engine resolves environmental bounds
        for (auto& state : entities) {
            real r = state.radius;
            if (state.position.y < -BOX_SIZE + r) { state.position.y = -BOX_SIZE + r; state.velocity.y *= -state.restitution; }
            if (state.position.y >  BOX_SIZE - r) { state.position.y =  BOX_SIZE - r; state.velocity.y *= -state.restitution; }
            if (state.position.x < -BOX_SIZE + r) { state.position.x = -BOX_SIZE + r; state.velocity.x *= -state.restitution; }
            if (state.position.x >  BOX_SIZE - r) { state.position.x =  BOX_SIZE - r; state.velocity.x *= -state.restitution; }
            if (state.position.z < -BOX_SIZE + r) { state.position.z = -BOX_SIZE + r; state.velocity.z *= -state.restitution; }
            if (state.position.z >  BOX_SIZE - r) { state.position.z =  BOX_SIZE - r; state.velocity.z *= -state.restitution; }
        }
    }

    std::cout << "Simulation completed purely functionally!" << std::endl;
    return 0;
}
