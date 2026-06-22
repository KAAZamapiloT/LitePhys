#pragma once
#include "../math/Vector4.h"

namespace Lite {

class Shape4D {
public:
    virtual ~Shape4D() = default;
    
    // Returns the furthest point on the shape in the given 4D direction
    virtual Vector4 getSupport(const Vector4& direction) const = 0;
};

// 4D Hyper-Sphere
struct Sphere4D : public Shape4D {
    Vector4 center;
    real radius;

    Sphere4D(const Vector4& center, real radius) : center(center), radius(radius) {}

    Vector4 getSupport(const Vector4& direction) const override {
        Vector4 dir = direction.normalized();
        return center + dir * radius;
    }
};

// 4D Hyper-Cube (Tesseract)
struct Hypercube4D : public Shape4D {
    Vector4 center;
    Vector4 halfExtents;

    Hypercube4D(const Vector4& center, const Vector4& halfExtents) : center(center), halfExtents(halfExtents) {}

    Vector4 getSupport(const Vector4& direction) const override {
        Vector4 result = center;
        result.x += (direction.x > 0) ? halfExtents.x : -halfExtents.x;
        result.y += (direction.y > 0) ? halfExtents.y : -halfExtents.y;
        result.z += (direction.z > 0) ? halfExtents.z : -halfExtents.z;
        result.w += (direction.w > 0) ? halfExtents.w : -halfExtents.w;
        return result;
    }
};

// 4D Spherocylinder (Capsule extended into 4D)
struct Spherocylinder4D : public Shape4D {
    Vector4 startPoint;
    Vector4 endPoint;
    real radius;

    Spherocylinder4D(const Vector4& start, const Vector4& end, real r) : startPoint(start), endPoint(end), radius(r) {}

    Vector4 getSupport(const Vector4& direction) const override {
        real dotStart = direction.dot(startPoint);
        real dotEnd = direction.dot(endPoint);

        Vector4 furthestPoint = (dotStart > dotEnd) ? startPoint : endPoint;
        Vector4 dir = direction.normalized();

        return furthestPoint + dir * radius;
    }
};

} // namespace Lite
