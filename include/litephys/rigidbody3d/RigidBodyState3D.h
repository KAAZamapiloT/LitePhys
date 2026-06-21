#pragma once
#include "../math/Vector3.h"
#include "../math/Quaternion.h"
#include "../math/Matrix3.h"

namespace Lite {

struct RigidBodyState3D {
    // Linear properties
    Vector3 position;
    Vector3 velocity;
    Vector3 force;

    real mass;
    real inverseMass;
    
    // Friction and Restitution
    real friction = 0.5f;
    real restitution = 0.5f;

    // Angular properties
    Quaternion orientation;
    Vector3 angularVelocity;
    Vector3 torque;

    // Inertia tensors
    Matrix3 inverseInertiaTensor;
    Matrix3 inverseInertiaTensorWorld;

    // Helpers
    void setMass(real m) {
        mass = m;
        if (m == 0) inverseMass = 0;
        else inverseMass = 1.0f / m;
    }

    void setInertiaTensor(const Matrix3& inertiaTensor) {
        inverseInertiaTensor = inertiaTensor.inverse();
    }
    
    // For spheres
    void setSphereInertia(real m, real radius) {
        setMass(m);
        real i = (2.0f / 5.0f) * m * radius * radius;
        Matrix3 tensor(
            i, 0, 0,
            0, i, 0,
            0, 0, i
        );
        setInertiaTensor(tensor);
    }
};

} // namespace Lite
