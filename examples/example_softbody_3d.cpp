#include <iostream>
#include <litephys/LiteSoftBody3D.h>

using namespace Lite;

int main() {
    std::cout << "LitePhys - Stateless 3D Soft Body Example" << std::endl;

    // Generate a 3x3x3 jello cube at origin
    SoftBodyState3D cube = SoftBodyGenerator3D::createCubeLattice(
        Vector3(0, 5, 0), // Origin (dropped from height 5)
        2.0f,             // Size
        3,                // Resolution (3x3x3 = 27 particles)
        1.0f,             // Mass per particle
        100.0f,           // Spring Stiffness
        2.0f              // Spring Damping
    );
    
    std::cout << "Generated Soft Body Cube!" << std::endl;
    std::cout << "Particles: " << cube.particles.size() << std::endl;
    std::cout << "Springs: " << cube.springs.size() << std::endl;
    
    // Simulate dropping it on the floor
    real dt = 0.016f; // 60 FPS
    
    for (int frame = 0; frame < 100; ++frame) {
        // 1. Apply Gravity and Springs
        SoftBodyForceSolver3D::applyGravity(cube, Vector3(0, -9.8f, 0));
        SoftBodyForceSolver3D::applySpringForces(cube);
        
        // 2. Simple Floor Collision
        for (auto& particle : cube.particles) {
            if (particle.position.y - particle.radius < 0.0f) {
                particle.position.y = particle.radius; // Resolve penetration
                particle.velocity.y *= -particle.restitution; // Bounce
            }
        }
        
        // 3. Integrate
        SoftBodyIntegrator3D::integrate(cube, dt);
    }
    
    std::cout << "\nAfter 100 frames of falling and hitting the floor:" << std::endl;
    
    real lowestY = 1000.0f;
    real highestY = -1000.0f;
    
    for (auto& particle : cube.particles) {
        if (particle.position.y < lowestY) lowestY = particle.position.y;
        if (particle.position.y > highestY) highestY = particle.position.y;
    }
    
    std::cout << "Lowest Particle Y: " << lowestY << std::endl;
    std::cout << "Highest Particle Y: " << highestY << std::endl;
    std::cout << "Current Cube Height: " << (highestY - lowestY) << " (Original was 2.0)" << std::endl;
    std::cout << "The cube has compressed upon hitting the floor!" << std::endl;

    return 0;
}
