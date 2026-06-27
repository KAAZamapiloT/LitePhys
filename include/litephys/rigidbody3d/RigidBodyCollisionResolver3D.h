#pragma once
#include "RigidBodyState3D.h"
#include "CollisionManifold3D.h"
#include <vector>
#include <algorithm>

namespace Lite {

struct ContactPair {
    RigidBodyState3D* stateA;
    RigidBodyState3D* stateB;
    CollisionManifold3D contact;
    
    // Cached for solver iterations
    Vector3 rA, rB;
    real normalMass;
    real tangentMass1;
    real tangentMass2;
    Vector3 tangent1, tangent2;
    real bias;
};

class RigidBodyCollisionResolver3D {
public:
    // Main Entry Point for Sequential Impulse Solver
    static void solveContacts(std::vector<ContactPair>& contacts, int iterations, real dt) {
        if (contacts.empty()) return;
        
        // 1. Pre-Step (Calculate masses, Baumgarte bias, and Warm Start)
        for (auto& pair : contacts) {
            preStep(pair, dt);
        }
        
        // 2. Iterative Solver
        for (int i = 0; i < iterations; ++i) {
            for (auto& pair : contacts) {
                resolveVelocity(pair);
            }
        }
    }

private:
    static void preStep(ContactPair& pair, real dt) {
        pair.rA = pair.contact.contactPoint - pair.stateA->position;
        pair.rB = pair.contact.contactPoint - pair.stateB->position;

        real invMassSum = pair.stateA->inverseMass + pair.stateB->inverseMass;

        // --- Normal Mass ---
        Vector3 crossA = pair.rA.cross(pair.contact.contactNormal);
        Vector3 crossB = pair.rB.cross(pair.contact.contactNormal);
        Vector3 inertiaA = (pair.stateA->inverseInertiaTensorWorld * crossA).cross(pair.rA);
        Vector3 inertiaB = (pair.stateB->inverseInertiaTensorWorld * crossB).cross(pair.rB);
        
        real angularEffect = (inertiaA + inertiaB).dot(pair.contact.contactNormal);
        pair.normalMass = invMassSum + angularEffect;

        // --- Baumgarte Stabilization (Bias) ---
        const real beta = 0.2f; // Penetration percentage to resolve per frame
        const real slop = 0.01f; // Penetration allowance
        pair.bias = -(beta / dt) * std::max(pair.contact.penetration - slop, 0.0f);

        // --- Tangent / Friction Setup ---
        // Create 2 orthogonal tangents for 3D friction
        Vector3 t1, t2;
        if (std::abs(pair.contact.contactNormal.x) >= 0.57735f) {
            t1 = Vector3(pair.contact.contactNormal.y, -pair.contact.contactNormal.x, 0.0f);
        } else {
            t1 = Vector3(0.0f, pair.contact.contactNormal.z, -pair.contact.contactNormal.y);
        }
        t1.normalize();
        t2 = pair.contact.contactNormal.cross(t1);

        pair.tangent1 = t1;
        pair.tangent2 = t2;
        
        // --- Warm Starting ---
        // Apply impulses accumulated from the previous frame
        Vector3 impulse = pair.contact.contactNormal * pair.contact.accumulatedNormalImpulse;
        impulse = impulse + pair.contact.accumulatedTangentImpulse;
        
        if (pair.contact.accumulatedNormalImpulse > 0.0f) {
            pair.stateA->velocity = pair.stateA->velocity + impulse * pair.stateA->inverseMass;
            pair.stateA->angularVelocity = pair.stateA->angularVelocity + pair.stateA->inverseInertiaTensorWorld * pair.rA.cross(impulse);
            
            pair.stateB->velocity = pair.stateB->velocity - impulse * pair.stateB->inverseMass;
            pair.stateB->angularVelocity = pair.stateB->angularVelocity - pair.stateB->inverseInertiaTensorWorld * pair.rB.cross(impulse);
        }
    }

    static void resolveVelocity(ContactPair& pair) {
        // --- Normal Impulse ---
        Vector3 vA = pair.stateA->velocity + pair.stateA->angularVelocity.cross(pair.rA);
        Vector3 vB = pair.stateB->velocity + pair.stateB->angularVelocity.cross(pair.rB);
        Vector3 relativeVelocity = vA - vB;
        
        real contactVelocity = relativeVelocity.dot(pair.contact.contactNormal);
        
        real restitution = std::min(pair.stateA->restitution, pair.stateB->restitution);
        real restitutionImpulse = 0.0f;
        // Only apply restitution if moving relatively fast (prevent jitter on resting)
        if (contactVelocity < -1.0f) {
            restitutionImpulse = -restitution * contactVelocity;
        }

        real j = (-contactVelocity + pair.bias + restitutionImpulse) / pair.normalMass;
        
        // Clamp accumulated impulse
        real oldNormalImpulse = pair.contact.accumulatedNormalImpulse;
        pair.contact.accumulatedNormalImpulse = std::max(oldNormalImpulse + j, 0.0f);
        j = pair.contact.accumulatedNormalImpulse - oldNormalImpulse;
        
        Vector3 impulse = pair.contact.contactNormal * j;
        applyImpulse(pair, impulse);

        // --- Friction (Tangent) Impulse ---
        vA = pair.stateA->velocity + pair.stateA->angularVelocity.cross(pair.rA);
        vB = pair.stateB->velocity + pair.stateB->angularVelocity.cross(pair.rB);
        relativeVelocity = vA - vB;
        
        // Resolve along tangent 1
        real jt1 = -relativeVelocity.dot(pair.tangent1);
        Vector3 crossA1 = pair.rA.cross(pair.tangent1);
        Vector3 crossB1 = pair.rB.cross(pair.tangent1);
        real tangentMass1 = pair.stateA->inverseMass + pair.stateB->inverseMass + 
                           ((pair.stateA->inverseInertiaTensorWorld * crossA1).cross(pair.rA) +
                            (pair.stateB->inverseInertiaTensorWorld * crossB1).cross(pair.rB)).dot(pair.tangent1);
        jt1 /= tangentMass1;

        // Resolve along tangent 2
        real jt2 = -relativeVelocity.dot(pair.tangent2);
        Vector3 crossA2 = pair.rA.cross(pair.tangent2);
        Vector3 crossB2 = pair.rB.cross(pair.tangent2);
        real tangentMass2 = pair.stateA->inverseMass + pair.stateB->inverseMass + 
                           ((pair.stateA->inverseInertiaTensorWorld * crossA2).cross(pair.rA) +
                            (pair.stateB->inverseInertiaTensorWorld * crossB2).cross(pair.rB)).dot(pair.tangent2);
        jt2 /= tangentMass2;

        // Coulomb Friction Clamp
        real mu = 0.5f; // Friction coefficient
        real maxFriction = mu * pair.contact.accumulatedNormalImpulse;
        
        // Calculate new accumulated tangent impulse (we accumulate it linearly for simplicity here)
        Vector3 oldTangentImpulse = pair.contact.accumulatedTangentImpulse;
        pair.contact.accumulatedTangentImpulse = oldTangentImpulse + pair.tangent1 * jt1 + pair.tangent2 * jt2;
        
        real tangentLength = pair.contact.accumulatedTangentImpulse.magnitude();
        if (tangentLength > maxFriction) {
            pair.contact.accumulatedTangentImpulse = pair.contact.accumulatedTangentImpulse * (maxFriction / tangentLength);
        }

        Vector3 frictionImpulse = pair.contact.accumulatedTangentImpulse - oldTangentImpulse;
        applyImpulse(pair, frictionImpulse);
    }

    static void applyImpulse(ContactPair& pair, const Vector3& impulse) {
        pair.stateA->velocity = pair.stateA->velocity + impulse * pair.stateA->inverseMass;
        pair.stateA->angularVelocity = pair.stateA->angularVelocity + pair.stateA->inverseInertiaTensorWorld * pair.rA.cross(impulse);
        
        pair.stateB->velocity = pair.stateB->velocity - impulse * pair.stateB->inverseMass;
        pair.stateB->angularVelocity = pair.stateB->angularVelocity - pair.stateB->inverseInertiaTensorWorld * pair.rB.cross(impulse);
    }
};

} // namespace Lite
