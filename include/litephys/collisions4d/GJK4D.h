#pragma once
#include "Shapes4D.h"
#include <vector>

namespace Lite {

struct Simplex4D {
    Vector4 points[5];
    int size = 0;

    void push_front(const Vector4& p) {
        points[4] = points[3];
        points[3] = points[2];
        points[2] = points[1];
        points[1] = points[0];
        points[0] = p;
        size = std::min(size + 1, 5);
    }
};

class GJK4D {
private:
    static Vector4 getSupport(const Shape4D& shapeA, const Shape4D& shapeB, const Vector4& direction) {
        return shapeA.getSupport(direction) - shapeB.getSupport(direction * -1.0f);
    }

public:
    // N-Dimensional GJK using the iterative Gilbert algorithm (Frank-Wolfe)
    static bool checkCollision(const Shape4D& shapeA, const Shape4D& shapeB, Simplex4D& simplex) {
        Vector4 v = getSupport(shapeA, shapeB, Vector4(1, 0, 0, 0));
        
        for (int i = 0; i < 100; ++i) { 
            // If v is at the origin, we have a collision
            if (v.squareMagnitude() < 0.0001f) {
                simplex.size = 1;
                simplex.points[0] = v;
                return true;
            }
            
            Vector4 dir = v * -1.0f;
            Vector4 w = getSupport(shapeA, shapeB, dir);
            
            // If the support point is not past the origin, the origin cannot be in the Minkowski Difference
            if (w.dot(dir) < 0) {
                return false; 
            }

            // Update v to be the closest point on the line segment between v and w
            Vector4 vw = w - v;
            real t = -v.dot(vw) / vw.dot(vw);
            t = std::max(0.0f, std::min(1.0f, t));
            v = v + vw * t;
            
            simplex.push_front(w);
        }
        
        return true; // Converged / enclosed origin
    }
};

} // namespace Lite
