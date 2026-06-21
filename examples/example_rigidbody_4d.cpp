#include <iostream>
#include <vector>
#include <litephys/LiteRigidBody4D.h>

using namespace Lite;

int main() {
    std::cout << "LitePhys - Stateless 4D Rigid Body Example" << std::endl;

    // 1. Set up a 4D rigid body hyper-sphere
    RigidBodyState4D body;
    body.position = Vector4(0, 10, 0, 5);
    body.velocity = Vector4(0, 0, 0, 0);
    body.setHyperSphereInertia(2.0f, 1.0f); // mass 2.0, radius 1.0
    
    // Calculate initial derived data (inertia tensor in world space)
    RigidBodyIntegrator4D::calculateDerivedData(body);

    real dt = 0.1f;

    std::cout << "Initial Position: (" << body.position.x << ", " << body.position.y << ", " << body.position.z << ", " << body.position.w << ")" << std::endl;
    std::cout << "Initial Angular Velocity (Bivector6): (" 
              << body.angularVelocity.xy << "xy, " 
              << body.angularVelocity.xz << "xz, "
              << body.angularVelocity.xw << "xw, "
              << body.angularVelocity.yz << "yz, "
              << body.angularVelocity.yw << "yw, "
              << body.angularVelocity.zw << "zw)" << std::endl;

    // Apply an OFF-CENTER impulse along the W-axis
    std::cout << "\nApplying an off-center impulse..." << std::endl;
    
    // Impulse pushing in +X direction
    Vector4 impulse(10.0f, 0, 0, 0); 
    // Point of impact is exactly on the edge of the W-axis (local +W)
    Vector4 impactPoint = body.position + Vector4(0, 0, 0, 1.0f);
    
    // Convert impulse to force
    Vector4 force = impulse * (1.0f / dt);
    
    // Apply force at the W offset
    RigidBodyForceSolver4D::applyForceAtPoint(body, force, impactPoint);

    // Simulate 1 frame
    RigidBodyIntegrator4D::integrate(body, dt);

    std::cout << "Position after 1 frame: (" << body.position.x << ", " << body.position.y << ", " << body.position.z << ", " << body.position.w << ")" << std::endl;
    std::cout << "Velocity after 1 frame: (" << body.velocity.x << ", " << body.velocity.y << ", " << body.velocity.z << ", " << body.velocity.w << ")" << std::endl;
    std::cout << "Angular Velocity after 1 frame: (" 
              << body.angularVelocity.xy << "xy, " 
              << body.angularVelocity.xz << "xz, "
              << body.angularVelocity.xw << "xw, "
              << body.angularVelocity.yz << "yz, "
              << body.angularVelocity.yw << "yw, "
              << body.angularVelocity.zw << "zw)" << std::endl;
    
    std::cout << "\nNotice how the angular velocity is perfectly isolated in the 'xw' plane!" << std::endl;
    std::cout << "Pushing a 4D sphere along X at an offset of W causes it to spin in the XW plane!" << std::endl;

    return 0;
}
