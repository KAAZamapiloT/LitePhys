#pragma once
#include "../particles3d/PhysicsState3D.h"
#include "SpatialHash3D.h"
#include <algorithm>

namespace Lite {

enum class CollisionMode {
    FAST_IMPULSE,
    STABLE_SPRING
};

class CollisionResolver3D {
public:
    static void resolve(std::vector<PhysicsState3D>& states, SpatialHash3D& spatialHash, CollisionMode mode = CollisionMode::FAST_IMPULSE, real springStiffness = 500.0f) {
        spatialHash.clear();
        for (auto& state : states) {
            spatialHash.insert(&state);
        }

        for (auto& stateA : states) {
            std::vector<PhysicsState3D*> neighbors = spatialHash.getNearby(stateA.position);

            for (auto* stateB : neighbors) {
                if (&stateA == stateB) continue;
                if (&stateA > stateB) continue; // Prevent double resolution

                Vector3 diff = stateA.position - stateB->position;
                real distSq = diff.dot(diff);
                real radiusSum = stateA.radius + stateB->radius;

                if (distSq < radiusSum * radiusSum && distSq > 0.0001f) {
                    real dist = Math::real_sqrt(distSq);
                    Vector3 normal = diff * (1.0f / dist);
                    real penetration = radiusSum - dist;

                    if (mode == CollisionMode::FAST_IMPULSE) {
                        real totalInverseMass = stateA.inverseMass + stateB->inverseMass;
                        if (totalInverseMass <= 0.0f) continue;
                        
                        // Positional correction
                        Vector3 correction = normal * (penetration / totalInverseMass);
                        stateA.position += correction * stateA.inverseMass;
                        stateB->position -= correction * stateB->inverseMass;

                        // Velocity impulse
                        Vector3 relVel = stateA.velocity - stateB->velocity;
                        real velAlongNormal = relVel.dot(normal);

                        if (velAlongNormal > 0) continue;

                        real e = std::min(stateA.restitution, stateB->restitution);
                        real j = -(1.0f + e) * velAlongNormal;
                        j /= totalInverseMass;

                        Vector3 impulse = normal * j;
                        stateA.velocity += impulse * stateA.inverseMass;
                        stateB->velocity -= impulse * stateB->inverseMass;
                    } 
                    else if (mode == CollisionMode::STABLE_SPRING) {
                        Vector3 force = normal * (penetration * springStiffness);
                        stateA.force += force;
                        stateB->force -= force;
                    }
                }
            }
        }
    }
};

} // namespace Lite
