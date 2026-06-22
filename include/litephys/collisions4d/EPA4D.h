#pragma once
#include "GJK4D.h"

namespace Lite {

class EPA4D {
public:
    // A simplified EPA approximation for 4D. 
    // True 4D EPA requires building a 4D polytope (tessellated into tetrahedra) and expanding it.
    // For this engine, we approximate penetration by expanding along the axes of the final GJK simplex.
    static std::pair<Vector4, real> getPenetration(const Shape4D& shapeA, const Shape4D& shapeB, const Simplex4D& simplex) {
        
        // Calculate an approximate centroid of the simplex
        Vector4 centroid;
        for (int i = 0; i < simplex.size; ++i) {
            centroid += simplex.points[i];
        }
        centroid *= (1.0f / simplex.size);

        // The direction from origin to centroid gives a rough collision normal
        Vector4 normal = centroid.normalized();
        
        // Sample the Minkowski Difference in this direction to find depth
        Vector4 p = shapeA.getSupport(normal) - shapeB.getSupport(normal * -1.0f);
        real depth = p.dot(normal);

        return {normal, std::abs(depth)};
    }
};

} // namespace Lite
