#include <iostream>
#include <vector>
#include <litephys/LiteRigidBody3D.h>

using namespace Lite;

// We use the new RigidBodyState3D to prove it works seamlessly!
int main() {
    std::cout << "LitePhys - Stateless 3D Rigid Body Example" << std::endl;

    // 1. Set up a rigid body sphere
    RigidBodyState3D body;
    body.position = Vector3(0, 10, 0);
    body.velocity = Vector3(0, 0, 0);
    body.setSphereInertia(2.0f, 1.0f); // mass 2.0, radius 1.0
    
    // Calculate initial derived data (inertia tensor in world space)
    RigidBodyIntegrator3D::calculateDerivedData(body);

    real dt = 0.1f;

    std::cout << "Initial Position: (" << body.position.x << ", " << body.position.y << ", " << body.position.z << ")" << std::endl;
    std::cout << "Initial Angular Velocity: (" << body.angularVelocity.x << ", " << body.angularVelocity.y << ", " << body.angularVelocity.z << ")" << std::endl;

    // Apply an OFF-CENTER impulse. This should generate both linear velocity AND angular velocity!
    std::cout << "\nApplying an off-center impulse..." << std::endl;
    
    // Impulse pushing in X direction
    Vector3 impulse(10.0f, 0, 0); 
    // Point of impact is on the top edge of the sphere (local +Y)
    Vector3 impactPoint = body.position + Vector3(0, 1.0f, 0);
    
    // In our library, an impulse is just a force applied over 1 dt
    // So force = impulse / dt
    Vector3 force = impulse * (1.0f / dt);
    
    RigidBodyForceSolver3D::applyForceAtPoint(body, force, impactPoint);

    // Simulate 1 frame
    RigidBodyIntegrator3D::integrate(body, dt);

    std::cout << "Position after 1 frame: (" << body.position.x << ", " << body.position.y << ", " << body.position.z << ")" << std::endl;
    std::cout << "Velocity after 1 frame: (" << body.velocity.x << ", " << body.velocity.y << ", " << body.velocity.z << ")" << std::endl;
    std::cout << "Angular Velocity after 1 frame: (" << body.angularVelocity.x << ", " << body.angularVelocity.y << ", " << body.angularVelocity.z << ")" << std::endl;
    
    std::cout << "\nNotice how the angular velocity is around the -Z axis? Pushing the top of a sphere in +X makes it roll forward!" << std::endl;

    return 0;
}
