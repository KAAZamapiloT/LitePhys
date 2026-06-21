#pragma once
#include "CollidableParticle.h"
#include "SpatialHash3D.h"

namespace Lite {

enum class CollisionMode {
    FAST_IMPULSE,
    STABLE_SPRING
};

class CollisionResolver3D {
public:
    CollisionMode mode;
    real springStiffness;

    CollisionResolver3D() : mode(CollisionMode::FAST_IMPULSE), springStiffness(500.0f) {}

    void resolve(std::vector<CollidableParticle*>& particles, SpatialHash3D& spatialHash) {
        spatialHash.clear();
        for (auto* p : particles) {
            spatialHash.insert(p);
        }

        for (auto* p1 : particles) {
            std::vector<CollidableParticle*> neighbors = spatialHash.getNearby(p1->getPosition());

            for (auto* p2 : neighbors) {
                if (p1 == p2) continue;
                if (p1 > p2) continue; // Prevent double resolution

                Vector3 diff = p1->getPosition() - p2->getPosition();
                real distSq = diff.dot(diff);
                real radiusSum = p1->radius + p2->radius;

                if (distSq < radiusSum * radiusSum && distSq > 0.0001f) {
                    real dist = Math::real_sqrt(distSq);
                    Vector3 normal = diff * (1.0f / dist);
                    real penetration = radiusSum - dist;

                    if (mode == CollisionMode::FAST_IMPULSE) {
                        real totalInverseMass = p1->getInverseMass() + p2->getInverseMass();
                        if (totalInverseMass <= 0.0f) continue;
                        
                        // Positional correction
                        Vector3 correction = normal * (penetration / totalInverseMass);
                        p1->setPosition(p1->getPosition() + correction * p1->getInverseMass());
                        p2->setPosition(p2->getPosition() - correction * p2->getInverseMass());

                        // Velocity impulse
                        Vector3 relVel = p1->getVelocity() - p2->getVelocity();
                        real velAlongNormal = relVel.dot(normal);

                        if (velAlongNormal > 0) continue;

                        real e = std::min(p1->restitution, p2->restitution);
                        real j = -(1.0f + e) * velAlongNormal;
                        j /= totalInverseMass;

                        Vector3 impulse = normal * j;
                        p1->setVelocity(p1->getVelocity() + impulse * p1->getInverseMass());
                        p2->setVelocity(p2->getVelocity() - impulse * p2->getInverseMass());
                    } 
                    else if (mode == CollisionMode::STABLE_SPRING) {
                        Vector3 force = normal * (penetration * springStiffness);
                        p1->addForce(force);
                        p2->addForce(force * -1.0f);
                    }
                }
            }
        }
    }
};

} // namespace Lite
