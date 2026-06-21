#pragma once
#include "RigidBodyState3D.h"
#include "CollisionManifold3D.h"
#include <vector>

namespace Lite {

class RigidBodyCollisionResolver3D {
public:
    // Resolve a single collision contact
    static void resolveContact(RigidBodyState3D& stateA, RigidBodyState3D& stateB, const CollisionManifold3D& contact) {
        // Find relative positions
        Vector3 rA = contact.contactPoint - stateA.position;
        Vector3 rB = contact.contactPoint - stateB.position;

        // Calculate relative velocity at contact point
        Vector3 vA = stateA.velocity + stateA.angularVelocity.cross(rA);
        Vector3 vB = stateB.velocity + stateB.angularVelocity.cross(rB);
        Vector3 relativeVelocity = vA - vB;

        real contactVelocity = relativeVelocity.dot(contact.contactNormal);

        // Do not resolve if objects are separating
        if (contactVelocity > 0) {
            return;
        }

        // Calculate restitution (bounciness)
        real e = std::min(stateA.restitution, stateB.restitution);

        // Calculate impulse scalar
        real invMassSum = stateA.inverseMass + stateB.inverseMass;
        if (invMassSum <= 0.0f) return;

        Vector3 crossA = rA.cross(contact.contactNormal);
        Vector3 crossB = rB.cross(contact.contactNormal);

        Vector3 inertiaA = (stateA.inverseInertiaTensorWorld * crossA).cross(rA);
        Vector3 inertiaB = (stateB.inverseInertiaTensorWorld * crossB).cross(rB);

        real angularEffect = (inertiaA + inertiaB).dot(contact.contactNormal);

        real j = -(1.0f + e) * contactVelocity;
        j = j / (invMassSum + angularEffect);

        // Apply impulse
        Vector3 impulse = contact.contactNormal * j;
        
        stateA.velocity = stateA.velocity + impulse * stateA.inverseMass;
        stateA.angularVelocity = stateA.angularVelocity + stateA.inverseInertiaTensorWorld * rA.cross(impulse);

        stateB.velocity = stateB.velocity - impulse * stateB.inverseMass;
        stateB.angularVelocity = stateB.angularVelocity - stateB.inverseInertiaTensorWorld * rB.cross(impulse);
        
        // Positional correction (Projection) to prevent sinking
        const real percent = 0.8f; // Penetration percentage to correct
        const real slop = 0.01f;   // Penetration allowance
        
        real correctionMag = std::max(contact.penetration - slop, 0.0f) / invMassSum * percent;
        Vector3 correction = contact.contactNormal * correctionMag;
        
        stateA.position = stateA.position + correction * stateA.inverseMass;
        stateB.position = stateB.position - correction * stateB.inverseMass;
    }
};

} // namespace Lite
