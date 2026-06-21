#pragma once
#include "RigidBodyState4D.h"
#include "CollisionManifold4D.h"
#include <vector>

namespace Lite {

class RigidBodyCollisionResolver4D {
public:
    // Resolve a single 4D collision contact
    static void resolveContact(RigidBodyState4D& stateA, RigidBodyState4D& stateB, const CollisionManifold4D& contact) {
        // Find relative positions
        Vector4 rA = contact.contactPoint - stateA.position;
        Vector4 rB = contact.contactPoint - stateB.position;

        // Calculate relative velocity at contact point using contraction (Bivector * Vector)
        Vector4 vA = stateA.velocity + stateA.angularVelocity.contract(rA);
        Vector4 vB = stateB.velocity + stateB.angularVelocity.contract(rB);
        Vector4 relativeVelocity = vA - vB;

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

        // In 4D, angular effect of the impulse on the contact normal relies on the inertia tensor
        // J_angular = (I^-1 * (r ^ n)) * r
        Bivector4 wedgeA = Bivector4::wedge(rA, contact.contactNormal);
        Bivector4 wedgeB = Bivector4::wedge(rB, contact.contactNormal);

        Bivector4 inertiaA_bivector = stateA.inverseInertiaTensorWorld * wedgeA;
        Bivector4 inertiaB_bivector = stateB.inverseInertiaTensorWorld * wedgeB;

        Vector4 angularVelA = inertiaA_bivector.contract(rA);
        Vector4 angularVelB = inertiaB_bivector.contract(rB);

        real angularEffect = (angularVelA + angularVelB).dot(contact.contactNormal);

        real j = -(1.0f + e) * contactVelocity;
        j = j / (invMassSum + angularEffect);

        // Apply 4D impulse
        Vector4 impulse = contact.contactNormal * j;
        
        stateA.velocity = stateA.velocity + impulse * stateA.inverseMass;
        stateA.angularVelocity = stateA.angularVelocity + stateA.inverseInertiaTensorWorld * Bivector4::wedge(rA, impulse);

        stateB.velocity = stateB.velocity - impulse * stateB.inverseMass;
        stateB.angularVelocity = stateB.angularVelocity - stateB.inverseInertiaTensorWorld * Bivector4::wedge(rB, impulse);
        
        // Positional correction (Projection) to prevent sinking
        const real percent = 0.8f; // Penetration percentage to correct
        const real slop = 0.01f;   // Penetration allowance
        
        real correctionMag = std::max(contact.penetration - slop, 0.0f) / invMassSum * percent;
        Vector4 correction = contact.contactNormal * correctionMag;
        
        stateA.position = stateA.position + correction * stateA.inverseMass;
        stateB.position = stateB.position - correction * stateB.inverseMass;
    }
};

} // namespace Lite
