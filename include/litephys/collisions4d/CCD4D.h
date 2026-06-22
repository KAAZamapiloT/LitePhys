#pragma once
#include "GJK4D.h"

namespace Lite {

class CCD4D {
public:
    // Performs Continuous Collision Detection using Binary Search
    // Returns true if a collision occurs within the time step (0.0 to 1.0)
    // tOut is the Time of Impact (TOI) [0.0, 1.0]
    template<typename ShapeTypeA, typename ShapeTypeB>
    static bool checkCollisionCCD(
        ShapeTypeA shapeA, const Vector4& velocityA,
        ShapeTypeB shapeB, const Vector4& velocityB,
        real& tOut) 
    {
        Simplex4D simplex;
        
        // Quick check: Are they colliding at t=0?
        if (GJK4D::checkCollision(shapeA, shapeB, simplex)) {
            tOut = 0.0f;
            return true;
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

            if (GJK4D::checkCollision(midA, midB, simplex)) {
                tMax = tMid;
                hit = true;
                lastHitTime = tMid;
            } else {
                if (hit) {
                    tMin = tMid;
                } else {
                    tMin = tMid; 
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
