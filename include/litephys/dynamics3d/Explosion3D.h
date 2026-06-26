#pragma once
#include "../rigidbody3d/RigidBodyState3D.h"
#include "../collisions3d/BVH3D.h"
#include "../math/Random.h"
#include <vector>

namespace Lite {

class Explosion3D {
public:
    // Applies an explosive radial impulse to all bodies within the blast radius.
    // Uses the BVH to quickly cull distant objects.
    // Adds a random angular velocity jitter to simulate chaotic debris spin.
    static void detonate(
        const Vector3& origin, 
        real radius, 
        real forceMagnitude, 
        const BVH3D& bvh, 
        std::vector<RigidBodyState3D>& bodies,
        Random& rng) 
    {
        // 1. Broadphase Query
        AABB3D blastBounds(
            Vector3(origin.x - radius, origin.y - radius, origin.z - radius),
            Vector3(origin.x + radius, origin.y + radius, origin.z + radius)
        );

        std::vector<int> candidates = bvh.queryAABB(blastBounds);

        // 2. Narrowphase Application
        real radiusSq = radius * radius;

        for (int index : candidates) {
            RigidBodyState3D& body = bodies[index];
            if (body.inverseMass <= 0.0f) continue; // Skip static bodies

            Vector3 toBody = body.position - origin;
            real distSq = toBody.squareMagnitude();

            if (distSq < radiusSq && distSq > 0.0001f) {
                real dist = std::sqrt(distSq);
                Vector3 direction = toBody * (1.0f / dist);

                // Inverse-square force attenuation: closer = stronger
                // Add a tiny epsilon to prevent infinite force at exactly center
                real attenuation = 1.0f - (dist / radius); 
                // Alternatively, real force = forceMagnitude / (distSq + 0.1f);
                
                real impulse = forceMagnitude * attenuation;

                // Apply linear impulse
                body.velocity += direction * (impulse * body.inverseMass);

                // Apply random angular jitter to simulate chaotic tumbling
                Vector3 angularJitter = rng.randomDirection() * (impulse * 0.1f);
                body.angularVelocity += body.inverseInertiaTensorWorld * angularJitter;
            }
        }
    }
};

} // namespace Lite
