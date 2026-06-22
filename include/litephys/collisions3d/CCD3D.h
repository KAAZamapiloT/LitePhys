#pragma once
#include "GJK3D.h"

namespace Lite {

class CCD3D {
public:
    // Performs Continuous Collision Detection using Binary Search
    // Returns true if a collision occurs within the time step (0.0 to 1.0)
    // tOut is the Time of Impact (TOI) [0.0, 1.0]
    template<typename ShapeTypeA, typename ShapeTypeB>
    static bool checkCollisionCCD(
        ShapeTypeA shapeA, const Vector3& velocityA,
        ShapeTypeB shapeB, const Vector3& velocityB,
        real& tOut) 
    {
        Simplex3D simplex;
        
        // Quick check: Are they colliding at t=0?
        if (GJK3D::checkCollision(shapeA, shapeB, simplex)) {
            tOut = 0.0f;
            return true;
        }

        // Quick check: Are they colliding at t=1?
        ShapeTypeA endA = shapeA;
        endA.center += velocityA;
        
        ShapeTypeB endB = shapeB;
        endB.center += velocityB;

        if (!GJK3D::checkCollision(endA, endB, simplex)) {
            // Not colliding at start or end. 
            // NOTE: Fast moving small objects could still pass through each other in the middle.
            // For a robust CCD, we check the midpoint if the swept AABBs overlap.
            // But we will binary search the sweep.
        }

        // Binary Search TOI
        real tMin = 0.0f;
        real tMax = 1.0f;
        bool hit = false;
        real lastHitTime = 1.0f;

        for (int i = 0; i < 12; ++i) { // 12 iterations gives 1/4096 precision
            real tMid = (tMin + tMax) * 0.5f;
            
            ShapeTypeA midA = shapeA;
            midA.center += velocityA * tMid;
            
            ShapeTypeB midB = shapeB;
            midB.center += velocityB * tMid;

            if (GJK3D::checkCollision(midA, midB, simplex)) {
                // Collision at tMid, search earlier in time
                tMax = tMid;
                hit = true;
                lastHitTime = tMid;
            } else {
                // No collision at tMid, if we had a hit before, it must be between tMid and tMax.
                // If we haven't hit yet, search later.
                if (hit) {
                    tMin = tMid;
                } else {
                    tMin = tMid; // We must search forward to find the first hit
                }
            }
        }

        if (hit) {
            tOut = lastHitTime;
            return true;
        }

        return false;
    }
};

} // namespace Lite
