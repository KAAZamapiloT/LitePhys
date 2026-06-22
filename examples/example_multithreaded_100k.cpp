#include <iostream>
#include <vector>
#include <chrono>
#include <litephys/LitePhys3D.h>
#include <litephys/particles3d/ParallelIntegrator3D.h>
#include <litephys/LiteCollisions3D.h>
#include <litephys/collisions3d/ParallelCollisionResolver3D.h>

using namespace Lite;

int main() {
    std::cout << "LitePhys - 100,000 Body Multithreaded Stress Test (C++23)" << std::endl;

    const int NUM_PARTICLES = 100000;
    std::vector<PhysicsState3D> states(NUM_PARTICLES);

    // Spawn 100k particles in a massive grid
    std::cout << "Spawning " << NUM_PARTICLES << " particles..." << std::endl;
    int side = std::cbrt(NUM_PARTICLES);
    int index = 0;
    for (int x = 0; x < side; ++x) {
        for (int y = 0; y < side; ++y) {
            for (int z = 0; z < side; ++z) {
                if (index >= NUM_PARTICLES) break;
                states[index].position = Vector3(x * 1.5f, y * 1.5f + 10.0f, z * 1.5f);
                states[index].radius = 0.5f;
                states[index].setMass(1.0f);
                states[index].restitution = 0.5f;
                index++;
            }
        }
    }

    SpatialHash3D spatialHash(1.5f); // Cell size of 1.5
    Vector3 gravity(0, -9.8f, 0);
    real dt = 0.016f;

    // Simulate 10 frames to measure performance
    std::cout << "Starting 10 frames of simulation..." << std::endl;
    
    auto startTime = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < 10; ++frame) {
        auto frameStart = std::chrono::high_resolution_clock::now();

        // 1. Parallel Gravity & Integration
        ParallelIntegrator3D::applyGravity(states, gravity);
        ParallelIntegrator3D::integrate(states, dt);

        // 2. Simple Floor Collision (Parallelized via execution::par_unseq)
        std::for_each(std::execution::par_unseq, states.begin(), states.end(), [](PhysicsState3D& state) {
            if (state.position.y - state.radius < 0.0f) {
                state.position.y = state.radius;
                state.velocity.y *= -state.restitution;
            }
        });

        // 3. Parallel Broad & Narrow Phase Collision Resolution
        // Using `sync = false` to embrace benign data races for maximum speed!
        ParallelCollisionResolver3D::resolve(states, spatialHash, false, CollisionMode::FAST_IMPULSE);

        auto frameEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> frameTime = frameEnd - frameStart;
        
        std::cout << "Frame " << frame << " took: " << frameTime.count() << " ms" << std::endl;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> totalTime = endTime - startTime;

    std::cout << "\nTest Complete!" << std::endl;
    std::cout << "Total Time for 10 frames: " << totalTime.count() << " ms" << std::endl;
    std::cout << "Average Time per frame: " << (totalTime.count() / 10.0) << " ms" << std::endl;

    return 0;
}
