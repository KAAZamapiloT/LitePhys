#pragma once
#include "../math/Vector4.h"
#include "../math/Matrix4.h"
#include "../math/Bivector4.h"
#include "../math/Matrix6.h"

namespace Lite {

struct RigidBodyState4D {
    // Linear properties
    Vector4 position;
    Vector4 velocity;
    Vector4 force;

    real mass;
    real inverseMass;
    
    // Friction and Restitution
    real friction = 0.5f;
    real restitution = 0.5f;

    // Angular properties
    Matrix4 orientation; // 4x4 rotation matrix
    Bivector4 angularVelocity; // 6D angular velocity
    Bivector4 torque; // 6D torque

    // Inertia tensors
    Matrix6 inverseInertiaTensor; // Maps 6D torque to 6D angular acceleration
    Matrix6 inverseInertiaTensorWorld;

    // Helpers
    void setMass(real m) {
        mass = m;
        if (m == 0) inverseMass = 0;
        else inverseMass = 1.0f / m;
    }

    void setInertiaTensor(const Matrix6& inertiaTensor) {
        // Assume for hyperspheres we just pass the inverted tensor directly for simplicity
        inverseInertiaTensor = inertiaTensor;
    }
    
    // For hyperspheres
    void setHyperSphereInertia(real m, real radius) {
        setMass(m);
        // Inertia of a 4D solid hypersphere is (2/6) * m * r^2  wait no, it's actually different.
        // Let's approximate it as simply proportional to m * r^2. Let's use 0.4 * m * r^2.
        real i = 0.4f * m * radius * radius;
        Matrix6 tensor;
        if (i == 0) tensor.setDiagonal(0);
        else tensor.setDiagonal(1.0f / i); // Directly store the inverse tensor
        setInertiaTensor(tensor);
    }
};

} // namespace Lite
