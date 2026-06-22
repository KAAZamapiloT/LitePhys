#pragma once
#include "WheelState3D.h"
#include "../rigidbody3d/RigidBodyState3D.h"
#include "../collisions3d/Ray3D.h"
#include "../collisions3d/BVH3D.h"
#include <vector>

namespace Lite {

class RaycastVehicle3D {
public:
    // User provides a raycast callback to the terrain/world
    // Returns true if hit, populates hitPoint and hitNormal
    typedef bool (*RaycastCallback)(const Ray3D& ray, real maxDist, Vector3& hitPoint, Vector3& hitNormal, void* userData);

    // Updates the vehicle chassis based on the wheel states
    static void updateVehicle(
        RigidBodyState3D& chassis, 
        std::vector<WheelState3D>& wheels, 
        real dt, 
        RaycastCallback raycastFunc, 
        void* userData) 
    {
        if (chassis.inverseMass <= 0.0f) return;

        for (auto& wheel : wheels) {
            // Transform local wheel vectors to world space
            Vector3 worldAttachPoint = chassis.position + chassis.orientation.rotate(wheel.localAttachmentPoint);
            Vector3 worldDir = chassis.orientation.rotate(wheel.localDirection).normalized();
            
            // Apply steering rotation around the local Y axis, then transform to world space
            Quaternion steeringRot = Quaternion::fromAxisAngle(Vector3(0,1,0), wheel.steeringAngle);
            Vector3 steeredAxle = steeringRot.rotate(wheel.localAxle);
            Vector3 worldAxle = chassis.orientation.rotate(steeredAxle).normalized();
            Vector3 forwardDir = worldAxle.cross(worldDir).normalized();

            // Cast Ray for suspension
            real maxCastDist = wheel.suspensionRestLength + wheel.wheelRadius;
            Ray3D ray(worldAttachPoint, worldDir);
            
            wheel.isGrounded = false;
            
            if (raycastFunc(ray, maxCastDist, wheel.hitPoint, wheel.hitNormal, userData)) {
                wheel.isGrounded = true;
                wheel.raycastDistance = (wheel.hitPoint - worldAttachPoint).magnitude();
                
                // Calculate suspension compression (0 = fully extended, >0 = compressed)
                real compression = (wheel.suspensionRestLength + wheel.wheelRadius) - wheel.raycastDistance;
                
                // Calculate velocity of chassis at the attachment point
                Vector3 r = worldAttachPoint - chassis.position;
                Vector3 pointVel = chassis.velocity + chassis.angularVelocity.cross(r);
                
                // Projected velocity along suspension direction
                real projectedVel = pointVel.dot(worldDir);
                
                // Hooke's Law for suspension: F = k*x - c*v
                real suspensionForce = (wheel.suspensionStiffness * compression) - (wheel.suspensionDamping * projectedVel);
                if (suspensionForce < 0.0f) suspensionForce = 0.0f; // Cannot pull the car down
                
                // Apply suspension force to chassis
                Vector3 forceVec = wheel.hitNormal * suspensionForce;
                
                // Simple integration of force directly to velocity for this frame
                chassis.velocity += forceVec * chassis.inverseMass * dt;
                chassis.angularVelocity += chassis.inverseInertiaTensor * r.cross(forceVec) * dt;

                // --- FRICTION & DRIVING ---
                
                // Recalculate point velocity after suspension
                pointVel = chassis.velocity + chassis.angularVelocity.cross(r);
                
                // Lateral slip (side-to-side)
                real sideVel = pointVel.dot(worldAxle);
                real slipImpulse = -sideVel * chassis.mass / wheels.size(); 
                // Clamp slip impulse by friction
                real maxFriction = suspensionForce * wheel.frictionSlip * dt;
                slipImpulse = std::max(-maxFriction, std::min(maxFriction, slipImpulse));
                
                Vector3 lateralForce = worldAxle * (slipImpulse / dt);

                // Longitudinal (forward/back)
                real forwardVel = pointVel.dot(forwardDir);
                real driveImpulse = wheel.engineForce * dt;
                
                // Braking
                if (wheel.brakeForce > 0.0f) {
                    real brakeImpulse = -forwardVel * chassis.mass / wheels.size();
                    real maxBrake = wheel.brakeForce * dt;
                    brakeImpulse = std::max(-maxBrake, std::min(maxBrake, brakeImpulse));
                    driveImpulse += brakeImpulse;
                }

                Vector3 longitudinalForce = forwardDir * (driveImpulse / dt);

                // Apply friction and drive forces at ground contact point
                Vector3 groundR = wheel.hitPoint - chassis.position;
                Vector3 totalSurfaceForce = lateralForce + longitudinalForce;

                chassis.velocity += totalSurfaceForce * chassis.inverseMass * dt;
                chassis.angularVelocity += chassis.inverseInertiaTensor * groundR.cross(totalSurfaceForce) * dt;
            } else {
                wheel.suspensionCompression = 0.0f;
            }
        }
    }
};

} // namespace Lite
