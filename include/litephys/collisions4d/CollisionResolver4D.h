#pragma once
#include "CollidableParticle4D.h"
#include "SpatialHash4D.h"
#include "../collisions3d/CollisionResolver3D.h" // For CollisionMode enum

namespace Lite {

class CollisionResolver4D {
public:
    CollisionMode mode;
    real springStiffness;

    CollisionResolver4D() : mode(CollisionMode::FAST_IMPULSE), springStiffness(500.0f) {}

    void resolve(std::vector<CollidableParticle4D*>& particles, SpatialHash4D& spatialHash) {
        spatialHash.clear();
        for (auto* p : particles) {
            spatialHash.insert(p);
        }

        for (auto* p1 : particles) {
            std::vector<CollidableParticle4D*> neighbors = spatialHash.getNearby(p1->getPosition());

            for (auto* p2 : neighbors) {
                if (p1 == p2) continue;
                if (p1 > p2) continue;

                Vector4 diff = p1->getPosition() - p2->getPosition();
                real distSq = diff.dot(diff);
                real radiusSum = p1->radius + p2->radius;

                if (distSq < radiusSum * radiusSum && distSq > 0.0001f) {
                    real dist = Math::real_sqrt(distSq);
                    Vector4 normal = diff * (1.0f / dist);
                    real penetration = radiusSum - dist;

                    if (mode == CollisionMode::FAST_IMPULSE) {
                        real totalInverseMass = p1->getInverseMass() + p2->getInverseMass();
                        if (totalInverseMass <= 0.0f) continue;
                        
                        Vector4 correction = normal * (penetration / totalInverseMass);
                        p1->setPosition(p1->getPosition() + correction * p1->getInverseMass());
                        p2->setPosition(p2->getPosition() - correction * p2->getInverseMass());

                        Vector4 relVel = p1->getVelocity() - p2->getVelocity();
                        real velAlongNormal = relVel.dot(normal);

                        if (velAlongNormal > 0) continue;

                        real e = std::min(p1->restitution, p2->restitution);
                        real j = -(1.0f + e) * velAlongNormal;
                        j /= totalInverseMass;

                        Vector4 impulse = normal * j;
                        p1->setVelocity(p1->getVelocity() + impulse * p1->getInverseMass());
                        p2->setVelocity(p2->getVelocity() - impulse * p2->getInverseMass());
                    } 
                    else if (mode == CollisionMode::STABLE_SPRING) {
                        Vector4 force = normal * (penetration * springStiffness);
                        p1->addForce(force);
                        p2->addForce(force * -1.0f);
                    }
                }
            }
        }
    }
};

} // namespace Lite
