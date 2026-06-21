#pragma once
#include "Ray3D.h"

namespace Lite {

struct Intersection3D {
    bool hit;
    real distance;
    Vector3 point;
    Vector3 normal;
    
    // Simple material properties for optics
    real refractiveIndex; // e.g., 1.0 for air, 1.5 for glass
    
    Intersection3D() : hit(false), distance(-1), refractiveIndex(1.0f) {}
};

class Shape3D {
public:
    real refractiveIndex = 1.5f;
    virtual ~Shape3D() {}
    virtual Intersection3D intersect(const Ray3D& ray) const = 0;
};

class Sphere3D : public Shape3D {
public:
    Vector3 center;
    real radius;

    Sphere3D(const Vector3& center, real radius) : center(center), radius(radius) {}

    virtual Intersection3D intersect(const Ray3D& ray) const override {
        Intersection3D result;
        Vector3 oc = ray.origin - center;
        real b = oc.dot(ray.direction);
        real c = oc.dot(oc) - radius * radius;
        real discriminant = b * b - c;

        if (discriminant > 0) {
            real sqrtDist = Math::real_sqrt(discriminant);
            real t1 = -b - sqrtDist;
            real t2 = -b + sqrtDist;

            if (t1 > 0.001f) {
                result.hit = true;
                result.distance = t1;
            } else if (t2 > 0.001f) {
                result.hit = true;
                result.distance = t2;
            }

            if (result.hit) {
                result.point = ray.origin + (ray.direction * result.distance);
                result.normal = (result.point - center).normalized();
                result.refractiveIndex = this->refractiveIndex;
            }
        }
        return result;
    }
};

class Plane3D : public Shape3D {
public:
    Vector3 normal;
    real d;

    Plane3D(const Vector3& normal, real d) : normal(normal.normalized()), d(d) {}

    virtual Intersection3D intersect(const Ray3D& ray) const override {
        Intersection3D result;
        real denom = normal.dot(ray.direction);
        if (Math::real_abs(denom) > 0.0001f) {
            real t = -(normal.dot(ray.origin) + d) / denom;
            if (t > 0.001f) {
                result.hit = true;
                result.distance = t;
                result.point = ray.origin + (ray.direction * t);
                result.normal = normal;
                result.refractiveIndex = this->refractiveIndex;
            }
        }
        return result;
    }
};

} // namespace Lite
