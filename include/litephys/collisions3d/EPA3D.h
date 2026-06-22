#pragma once
#include "GJK3D.h"
#include <vector>
#include <limits>

namespace Lite {

struct EPAFace {
    int a, b, c;
    Vector3 normal;
    real distance;
};

class EPA3D {
private:
    static EPAFace makeFace(const std::vector<Vector3>& polytope, int a, int b, int c) {
        Vector3 ab = polytope[b] - polytope[a];
        Vector3 ac = polytope[c] - polytope[a];
        Vector3 normal = ab.cross(ac).normalized();
        real distance = normal.dot(polytope[a]);

        if (distance < 0) {
            normal = normal * -1.0f;
            distance = -distance;
            return {c, b, a, normal, distance};
        }
        return {a, b, c, normal, distance};
    }

public:
    // Takes the simplex from GJK, expands it, and returns {Normal, Penetration Depth}
    static std::pair<Vector3, real> getPenetration(const Shape3D& shapeA, const Shape3D& shapeB, const Simplex3D& simplex) {
        std::vector<Vector3> polytope(simplex.points, simplex.points + 4);
        std::vector<EPAFace> faces;

        faces.push_back(makeFace(polytope, 0, 1, 2));
        faces.push_back(makeFace(polytope, 0, 2, 3));
        faces.push_back(makeFace(polytope, 0, 3, 1));
        faces.push_back(makeFace(polytope, 1, 3, 2));

        for (int iterations = 0; iterations < 64; ++iterations) {
            // Find closest face to origin
            int closestFaceIndex = 0;
            real minDistance = std::numeric_limits<real>::max();
            for (size_t i = 0; i < faces.size(); ++i) {
                if (faces[i].distance < minDistance) {
                    minDistance = faces[i].distance;
                    closestFaceIndex = i;
                }
            }

            EPAFace closestFace = faces[closestFaceIndex];
            Vector3 searchDirection = closestFace.normal;
            
            // Get furthest point in this direction
            Vector3 p = shapeA.getSupport(searchDirection) - shapeB.getSupport(searchDirection * -1.0f);
            
            real pDist = p.dot(searchDirection);

            // If the point is not significantly further than the face, we found the boundary
            if (pDist - minDistance < 0.001f) {
                return {closestFace.normal, pDist};
            }

            // Remove faces that can "see" this new point
            std::vector<std::pair<int, int>> edges;
            for (auto it = faces.begin(); it != faces.end();) {
                if (it->normal.dot(p - polytope[it->a]) > 0) {
                    // Add edges
                    edges.push_back({it->a, it->b});
                    edges.push_back({it->b, it->c});
                    edges.push_back({it->c, it->a});
                    it = faces.erase(it);
                } else {
                    ++it;
                }
            }

            // Keep only unique edges (edges appearing once)
            std::vector<std::pair<int, int>> uniqueEdges;
            for (size_t i = 0; i < edges.size(); ++i) {
                bool isUnique = true;
                for (size_t j = 0; j < edges.size(); ++j) {
                    if (i != j && edges[i].first == edges[j].second && edges[i].second == edges[j].first) {
                        isUnique = false;
                        break;
                    }
                }
                if (isUnique) {
                    uniqueEdges.push_back(edges[i]);
                }
            }

            // Add new point to polytope
            int newPointIndex = polytope.size();
            polytope.push_back(p);

            // Rebuild faces from unique edges to the new point
            for (const auto& edge : uniqueEdges) {
                faces.push_back(makeFace(polytope, edge.first, edge.second, newPointIndex));
            }
        }

        // Fallback if EPA doesn't converge
        return {faces[0].normal, faces[0].distance};
    }
};

} // namespace Lite
