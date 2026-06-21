#pragma once
#include "../particles4d/PhysicsState4D.h"
#include "SpatialHash4D.h"
#include "../collisions3d/CollisionResolver3D.h" // For CollisionMode enum
#include <algorithm>

namespace Lite {

class CollisionResolver4D {
public:
    static void resolve(std::vector<PhysicsState4D>& states, SpatialHash4D& spatialHash, CollisionMode mode = CollisionMode::FAST_IMPULSE, real springStiffness = 500.0f) {
        spatialHash.clear();
        for (auto& state : states) {
            spatialHash.insert(&state);
        }

        for (auto& stateA : states) {
            std::vector<PhysicsState4D*> neighbors = spatialHash.getNearby(stateA.position);

            for (auto* stateB : neighbors) {
                if (&stateA == stateB) continue;
                if (&stateA > stateB) continue; // Prevent double resolution

                Vector4 diff = stateA.position - stateB->position;
                real distSq = diff.dot(diff);
                real radiusSum = stateA.radius + stateB->radius;

                if (distSq < radiusSum * radiusSum && distSq > 0.0001f) {
                    real dist = Math::real_sqrt(distSq);
                    Vector4 normal = diff * (1.0f / dist);
                    real penetration = radiusSum - dist;

                    if (mode == CollisionMode::FAST_IMPULSE) {
                        real totalInverseMass = stateA.inverseMass + stateB->inverseMass;
                        if (totalInverseMass <= 0.0f) continue;
                        
                        // Positional correction
                        Vector4 correction = normal * (penetration / totalInverseMass);
                        stateA.position += correction * stateA.inverseMass;
                        stateB->position -= correction * stateB->inverseMass;

                        // Velocity impulse
                        Vector4 relVel = stateA.velocity - stateB->velocity;
                        real velAlongNormal = relVel.dot(normal);

                        if (velAlongNormal > 0) continue;

                        real e = std::min(stateA.restitution, stateB->restitution);
                        real j = -(1.0f + e) * velAlongNormal;
                        j /= totalInverseMass;

                        Vector4 impulse = normal * j;
                        stateA.velocity += impulse * stateA.inverseMass;
                        stateB->velocity -= impulse * stateB->inverseMass;
                    } 
                    else if (mode == CollisionMode::STABLE_SPRING) {
                        Vector4 force = normal * (penetration * springStiffness);
                        stateA.force += force;
                        stateB->force -= force;
                    }
                }
            }
        }
    }
};

} // namespace Lite
