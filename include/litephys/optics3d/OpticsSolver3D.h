#pragma once
#include "Geometry3D.h"
#include <vector>

namespace Lite {

struct Bounce3D {
    Vector3 point;
    Vector3 direction; 
    bool isRefraction;
};

class OpticsSolver3D {
    std::vector<Shape3D*> shapes;

public:
    void addShape(Shape3D* shape) {
        shapes.push_back(shape);
    }

    // Refract a vector using Snell's law
    static Vector3 refract(const Vector3& I, const Vector3& N, real ior) {
        real cosi = Math::real_abs(I.dot(N));
        real etai = 1.0f, etat = ior;
        Vector3 n = N;
        if (I.dot(N) < 0) {
            cosi = -I.dot(N);
        } else {
            // Inside the material, going out
            std::swap(etai, etat);
            n = N * -1.0f;
        }
        real eta = etai / etat;
        real k = 1.0f - eta * eta * (1.0f - cosi * cosi);
        if (k < 0.0f) {
            // Total internal reflection
            return reflect(I, n);
        } else {
            return I * eta + n * (eta * cosi - Math::real_sqrt(k));
        }
    }

    static Vector3 reflect(const Vector3& I, const Vector3& N) {
        return I - N * 2.0f * I.dot(N);
    }

    // Trace a ray and return a path of bounces
    std::vector<Bounce3D> traceRay(Ray3D ray, int maxBounces) const {
        std::vector<Bounce3D> path;
        
        for (int b = 0; b < maxBounces; ++b) {
            Intersection3D closest;
            closest.distance = std::numeric_limits<real>::max();

            for (auto* shape : shapes) {
                Intersection3D hit = shape->intersect(ray);
                if (hit.hit && hit.distance < closest.distance) {
                    closest = hit;
                }
            }

            if (!closest.hit) break;

            Bounce3D bounce;
            bounce.point = closest.point;
            
            // Simplification: We will just refract always for transparent bodies
            bounce.direction = refract(ray.direction, closest.normal, closest.refractiveIndex);
            bounce.isRefraction = true;
            
            path.push_back(bounce);
            
            // Setup next ray
            ray.origin = closest.point;
            ray.direction = bounce.direction;
        }

        return path;
    }
};

} // namespace Lite
