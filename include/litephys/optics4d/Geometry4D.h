#pragma once
#include "Ray4D.h"

namespace Lite {

struct Intersection4D {
    bool hit;
    real distance;
    Vector4 point;
    Vector4 normal;
    real refractiveIndex;
    
    Intersection4D() : hit(false), distance(-1), refractiveIndex(1.0f) {}
};

class Shape4D {
public:
    real refractiveIndex = 1.5f;
    virtual ~Shape4D() {}
    virtual Intersection4D intersect(const Ray4D& ray) const = 0;
};

class HyperSphere4D : public Shape4D {
public:
    Vector4 center;
    real radius;

    HyperSphere4D(const Vector4& center, real radius) : center(center), radius(radius) {}

    virtual Intersection4D intersect(const Ray4D& ray) const override {
        Intersection4D result;
        Vector4 oc = ray.origin - center;
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

class HyperPlane4D : public Shape4D {
public:
    Vector4 normal;
    real d;

    HyperPlane4D(const Vector4& normal, real d) : normal(normal.normalized()), d(d) {}

    virtual Intersection4D intersect(const Ray4D& ray) const override {
        Intersection4D result;
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
