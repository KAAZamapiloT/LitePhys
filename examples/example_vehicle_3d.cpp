#include <iostream>
#include <vector>
#include <litephys/rigidbody3d/RigidBodyState3D.h>
#include <litephys/vehicles3d/RaycastVehicle3D.h>
#include <litephys/collisions3d/BVH3D.h>

using namespace Lite;

// Global BVH for the terrain
BVH3D terrainBVH;

// A simple ground plane at Y=0
bool simpleGroundRaycast(const Ray3D& ray, real maxDist, Vector3& hitPoint, Vector3& hitNormal, void* userData) {
    if (ray.direction.y >= -0.0001f) return false; // Not pointing down

    // Ray: origin.y + direction.y * t = 0
    real t = -ray.origin.y / ray.direction.y;
    
    if (t >= 0 && t <= maxDist) {
        hitPoint = ray.getPoint(t);
        hitNormal = Vector3(0, 1, 0);
        return true;
    }
    return false;
}

int main() {
    std::cout << "LitePhys - Raycast Vehicle 3D Simulation\n" << std::endl;

    // 1. Create the Chassis (Rigid Body)
    RigidBodyState3D chassis;
    chassis.position = Vector3(0, 2.0f, 0); // Start dropped in the air
    chassis.mass = 1500.0f; // 1500 kg car
    chassis.inverseMass = 1.0f / chassis.mass;
    chassis.setBoxInertia(Vector3(1.0f, 0.5f, 2.0f)); // 2m wide, 1m tall, 4m long car

    // 2. Create the Wheels
    std::vector<WheelState3D> wheels;
    
    // Front Left
    WheelState3D fl(Vector3(-1.0f, -0.5f, 2.0f));
    // Front Right
    WheelState3D fr(Vector3(1.0f, -0.5f, 2.0f));
    // Rear Left
    WheelState3D rl(Vector3(-1.0f, -0.5f, -2.0f));
    // Rear Right
    WheelState3D rr(Vector3(1.0f, -0.5f, -2.0f));

    wheels.push_back(fl);
    wheels.push_back(fr);
    wheels.push_back(rl);
    wheels.push_back(rr);

    real dt = 1.0f / 60.0f;

    std::cout << "Simulating 60 frames of the car dropping onto the ground..." << std::endl;

    for (int frame = 0; frame < 60; ++frame) {
        // Gravity
        chassis.velocity.y += -9.81f * dt;

        // Accelerate
        if (frame > 30) {
            wheels[2].engineForce = 5000.0f; // Rear wheel drive
            wheels[3].engineForce = 5000.0f;
        }

        // Steer
        if (frame > 45) {
            wheels[0].steeringAngle = 0.5f; // Turn right
            wheels[1].steeringAngle = 0.5f;
        }

        // Vehicle Controller updates suspension and friction impulses
        RaycastVehicle3D::updateVehicle(chassis, wheels, dt, simpleGroundRaycast, nullptr);

        // Integrate Chassis
        chassis.position += chassis.velocity * dt;
        
        // Semi-implicit orientation integration
        Quaternion spin = Quaternion(chassis.angularVelocity.x, chassis.angularVelocity.y, chassis.angularVelocity.z, 0.0f);
        chassis.orientation += (spin * chassis.orientation) * (dt * 0.5f);
        chassis.orientation.normalize();

        if (frame % 10 == 0) {
            std::cout << "Frame " << frame 
                      << " | Pos: Y=" << chassis.position.y << ", Z=" << chassis.position.z 
                      << " | Vel: " << chassis.velocity.magnitude() 
                      << " | Grounded: " << (wheels[0].isGrounded ? "Yes" : "No") << std::endl;
        }
    }

    std::cout << "\nFinal Pos: Y=" << chassis.position.y << ", Z=" << chassis.position.z << std::endl;
    std::cout << "Notice how the chassis Y stabilizes at ~1.0 due to suspension springs resting," << std::endl;
    std::cout << "and the Z position increases rapidly after frame 30 due to engine drive force!" << std::endl;

    return 0;
}
