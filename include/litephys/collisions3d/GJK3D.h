#pragma once
#include "Shapes3D.h"
#include <vector>

namespace Lite {

// Represents a set of points (up to 4 in 3D)
struct Simplex3D {
    Vector3 points[4];
    int size = 0;

    void push_front(const Vector3& p) {
        points[3] = points[2];
        points[2] = points[1];
        points[1] = points[0];
        points[0] = p;
        size = std::min(size + 1, 4);
    }
};

class GJK3D {
private:
    static Vector3 getSupport(const Shape3D& shapeA, const Shape3D& shapeB, const Vector3& direction) {
        return shapeA.getSupport(direction) - shapeB.getSupport(direction * -1.0f);
    }

    static bool handleLine(Simplex3D& simplex, Vector3& direction) {
        Vector3 a = simplex.points[0];
        Vector3 b = simplex.points[1];

        Vector3 ab = b - a;
        Vector3 ao = a * -1.0f;

        if (ab.dot(ao) > 0) {
            direction = ab.cross(ao).cross(ab);
        } else {
            simplex.size = 1;
            direction = ao;
        }
        return false;
    }

    static bool handleTriangle(Simplex3D& simplex, Vector3& direction) {
        Vector3 a = simplex.points[0];
        Vector3 b = simplex.points[1];
        Vector3 c = simplex.points[2];

        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ao = a * -1.0f;

        Vector3 abc = ab.cross(ac);

        if (abc.cross(ac).dot(ao) > 0) {
            if (ac.dot(ao) > 0) {
                simplex.points[1] = c;
                simplex.size = 2;
                direction = ac.cross(ao).cross(ac);
            } else {
                return handleLine(simplex, direction);
            }
        } else {
            if (ab.cross(abc).dot(ao) > 0) {
                return handleLine(simplex, direction);
            } else {
                if (abc.dot(ao) > 0) {
                    direction = abc;
                } else {
                    simplex.points[1] = c;
                    simplex.points[2] = b;
                    direction = abc * -1.0f;
                }
            }
        }
        return false;
    }

    static bool handleTetrahedron(Simplex3D& simplex, Vector3& direction) {
        Vector3 a = simplex.points[0];
        Vector3 b = simplex.points[1];
        Vector3 c = simplex.points[2];
        Vector3 d = simplex.points[3];

        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ad = d - a;
        Vector3 ao = a * -1.0f;

        Vector3 abc = ab.cross(ac);
        Vector3 acd = ac.cross(ad);
        Vector3 adb = ad.cross(ab);

        if (abc.dot(ao) > 0) {
            simplex.size = 3;
            return handleTriangle(simplex, direction);
        }

        if (acd.dot(ao) > 0) {
            simplex.points[1] = c;
            simplex.points[2] = d;
            simplex.size = 3;
            return handleTriangle(simplex, direction);
        }

        if (adb.dot(ao) > 0) {
            simplex.points[1] = d;
            simplex.points[2] = b;
            simplex.size = 3;
            return handleTriangle(simplex, direction);
        }

        return true; // Origin is inside the tetrahedron
    }

    static bool handleSimplex(Simplex3D& simplex, Vector3& direction) {
        if (simplex.size == 2) {
            return handleLine(simplex, direction);
        } else if (simplex.size == 3) {
            return handleTriangle(simplex, direction);
        } else if (simplex.size == 4) {
            return handleTetrahedron(simplex, direction);
        }
        return false;
    }

public:
    // Returns true if shapeA and shapeB intersect, false otherwise.
    // If they intersect, `simplex` is populated with the final simplex, which can be passed to EPA.
    static bool checkCollision(const Shape3D& shapeA, const Shape3D& shapeB, Simplex3D& simplex) {
        Vector3 direction(1, 0, 0); // Arbitrary initial direction
        Vector3 support = getSupport(shapeA, shapeB, direction);
        
        simplex.push_front(support);
        direction = support * -1.0f;

        for (int i = 0; i < 64; ++i) { // Safety limit to prevent infinite loops
            support = getSupport(shapeA, shapeB, direction);
            
            // If the furthest point in the search direction does not pass the origin, we can't possibly enclose it
            if (support.dot(direction) <= 0) {
                return false; // No collision
            }

            simplex.push_front(support);

            if (handleSimplex(simplex, direction)) {
                return true; // Collision detected
            }
        }
        
        return false;
    }
};

} // namespace Lite
