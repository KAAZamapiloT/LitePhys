#include <iostream>
#include <litephys/LiteSoftBody4D.h>

using namespace Lite;

int main() {
    std::cout << "LitePhys - Stateless 4D Soft Body Example" << std::endl;

    // Generate a 3x3x3x3 jello tesseract at origin
    SoftBodyState4D tesseract = SoftBodyGenerator4D::createTesseractLattice(
        Vector4(0, 5, 0, 0), // Origin (dropped from Y=5)
        2.0f,                // Size
        3,                   // Resolution (3x3x3x3 = 81 particles)
        1.0f,                // Mass per particle
        100.0f,              // Spring Stiffness
        2.0f                 // Spring Damping
    );
    
    std::cout << "Generated Soft Body Tesseract!" << std::endl;
    std::cout << "Particles: " << tesseract.particles.size() << std::endl;
    std::cout << "Springs: " << tesseract.springs.size() << std::endl;
    
    // Simulate dropping it on the 3D hyperplane (floor)
    real dt = 0.016f; // 60 FPS
    
    for (int frame = 0; frame < 100; ++frame) {
        // 1. Apply Gravity and Springs
        SoftBodyForceSolver4D::applyGravity(tesseract, Vector4(0, -9.8f, 0, 0));
        SoftBodyForceSolver4D::applySpringForces(tesseract);
        
        // 2. Simple Floor Collision (Y=0 hyperplane)
        for (auto& particle : tesseract.particles) {
            if (particle.position.y - particle.radius < 0.0f) {
                particle.position.y = particle.radius; // Resolve penetration
                particle.velocity.y *= -particle.restitution; // Bounce
            }
        }
        
        // 3. Integrate
        SoftBodyIntegrator4D::integrate(tesseract, dt);
    }
    
    std::cout << "\nAfter 100 frames of falling and hitting the 3D floor hyperplane:" << std::endl;
    
    real lowestY = 1000.0f;
    real highestY = -1000.0f;
    
    for (auto& particle : tesseract.particles) {
        if (particle.position.y < lowestY) lowestY = particle.position.y;
        if (particle.position.y > highestY) highestY = particle.position.y;
    }
    
    std::cout << "Lowest Particle Y: " << lowestY << std::endl;
    std::cout << "Highest Particle Y: " << highestY << std::endl;
    std::cout << "Current Tesseract Y-Height: " << (highestY - lowestY) << " (Original was 2.0)" << std::endl;
    std::cout << "The 4D tesseract has compressed along the Y axis!" << std::endl;

    return 0;
}
