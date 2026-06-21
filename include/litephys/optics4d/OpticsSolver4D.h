#pragma once
#include "Geometry4D.h"
#include <vector>

namespace Lite {

struct Bounce4D {
    Vector4 point;
    Vector4 direction;
    bool isRefraction;
};

class OpticsSolver4D {
    std::vector<Shape4D*> shapes;

public:
    void addShape(Shape4D* shape) {
        shapes.push_back(shape);
    }

    static Vector4 refract(const Vector4& I, const Vector4& N, real ior) {
        real cosi = Math::real_abs(I.dot(N));
        real etai = 1.0f, etat = ior;
        Vector4 n = N;
        if (I.dot(N) < 0) {
            cosi = -I.dot(N);
        } else {
            std::swap(etai, etat);
            n = N * -1.0f;
        }
        real eta = etai / etat;
        real k = 1.0f - eta * eta * (1.0f - cosi * cosi);
        if (k < 0.0f) {
            return reflect(I, n);
        } else {
            return I * eta + n * (eta * cosi - Math::real_sqrt(k));
        }
    }

    static Vector4 reflect(const Vector4& I, const Vector4& N) {
        return I - N * 2.0f * I.dot(N);
    }

    std::vector<Bounce4D> traceRay(Ray4D ray, int maxBounces) const {
        std::vector<Bounce4D> path;
        
        for (int b = 0; b < maxBounces; ++b) {
            Intersection4D closest;
            closest.distance = std::numeric_limits<real>::max();

            for (auto* shape : shapes) {
                Intersection4D hit = shape->intersect(ray);
                if (hit.hit && hit.distance < closest.distance) {
                    closest = hit;
                }
            }

            if (!closest.hit) break;

            Bounce4D bounce;
            bounce.point = closest.point;
            bounce.direction = refract(ray.direction, closest.normal, closest.refractiveIndex);
            bounce.isRefraction = true;
            
            path.push_back(bounce);
            
            ray.origin = closest.point;
            ray.direction = bounce.direction;
        }

        return path;
    }
};

} // namespace Lite
