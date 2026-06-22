#pragma once
#include "../particles3d/PhysicsState3D.h"
#include "SpatialHash3D.h"
#include "CollisionResolver3D.h"
#include <vector>
#include <execution>
#include <algorithm>
#include <mutex>

namespace Lite {

class ParallelCollisionResolver3D {
public:
    // A multithreaded resolver that accepts benign race conditions if sync = false, 
    // or uses standard mutex locking if sync = true.
    static void resolve(std::vector<PhysicsState3D>& states, SpatialHash3D& spatialHash, bool sync = false, CollisionMode mode = CollisionMode::FAST_IMPULSE) {
        
        // 1. Populate the Spatial Hash (Sequential, as std::unordered_map is not thread-safe for insertions)
        spatialHash.clear();
        for (auto& state : states) {
            spatialHash.insert(&state);
        }

        std::mutex collisionMutex;

        // 2. Resolve Collisions in Parallel
        // We iterate over every particle concurrently.
        std::for_each(std::execution::par_unseq, states.begin(), states.end(), [&](PhysicsState3D& stateA) {
            
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
                        
                        Vector3 correction = normal * (penetration / totalInverseMass);
                        
                        // Velocity impulse
                        Vector3 relVel = stateA.velocity - stateB->velocity;
                        real velAlongNormal = relVel.dot(normal);

                        if (velAlongNormal > 0) continue;

                        real e = std::min(stateA.restitution, stateB->restitution);
                        real j = -(1.0f + e) * velAlongNormal;
                        j /= totalInverseMass;

                        Vector3 impulse = normal * j;
                        
                        if (sync) {
                            // Safe mode (Locked)
                            std::lock_guard<std::mutex> lock(collisionMutex);
                            stateA.position += correction * stateA.inverseMass;
                            stateB->position -= correction * stateB->inverseMass;
                            stateA.velocity += impulse * stateA.inverseMass;
                            stateB->velocity -= impulse * stateB->inverseMass;
                        } else {
                            // Ultra-Fast Mode (Embraces benign race conditions!)
                            stateA.position += correction * stateA.inverseMass;
                            stateB->position -= correction * stateB->inverseMass;
                            stateA.velocity += impulse * stateA.inverseMass;
                            stateB->velocity -= impulse * stateB->inverseMass;
                        }
                    } 
                    else if (mode == CollisionMode::STABLE_SPRING) {
                        Vector3 force = normal * (penetration * 500.0f);
                        if (sync) {
                            std::lock_guard<std::mutex> lock(collisionMutex);
                            stateA.force += force;
                            stateB->force -= force;
                        } else {
                            stateA.force += force;
                            stateB->force -= force;
                        }
                    }
                }
            }
        });
    }
};

} // namespace Lite
