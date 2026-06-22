#pragma once
#include "../math/Vector3.h"

namespace Lite {

struct WheelState3D {
    // Configuration
    Vector3 localAttachmentPoint; // Where the suspension connects to the chassis
    Vector3 localDirection;       // Usually (0, -1, 0)
    Vector3 localAxle;            // Usually (-1, 0, 0) or (1, 0, 0)
    
    real suspensionRestLength = 0.5f;
    real wheelRadius = 0.3f;
    
    // Suspension tuning
    real suspensionStiffness = 50.0f;
    real suspensionDamping = 4.0f;
    
    // Friction
    real frictionSlip = 10.5f;
    
    // Inputs
    real steeringAngle = 0.0f; // In radians
    real engineForce = 0.0f;   // Forward push
    real brakeForce = 0.0f;    // Braking
    
    // State output (updated by the solver)
    bool isGrounded = false;
    real suspensionCompression = 0.0f;
    real raycastDistance = 0.0f;
    Vector3 hitNormal;
    Vector3 hitPoint;

    WheelState3D() {}
    WheelState3D(const Vector3& attachPt) : localAttachmentPoint(attachPt) {
        localDirection = Vector3(0, -1, 0);
        localAxle = Vector3(-1, 0, 0);
    }
};

} // namespace Lite
