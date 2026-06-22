#pragma once
#include "../math/Vector3.h"

namespace Lite {

// Base class or concept for GJK Shapes
class Shape3D {
public:
    virtual ~Shape3D() = default;
    
    // Returns the furthest point on the shape in the given direction
    virtual Vector3 getSupport(const Vector3& direction) const = 0;
};

// Sphere Shape
struct Sphere3D : public Shape3D {
    Vector3 center;
    real radius;

    Sphere3D(const Vector3& center, real radius) : center(center), radius(radius) {}

    Vector3 getSupport(const Vector3& direction) const override {
        Vector3 dir = direction.normalized();
        return center + dir * radius;
    }
};

// Box Shape (AABB or OBB if center/orientation are tracked outside)
struct Box3D : public Shape3D {
    Vector3 center;
    Vector3 halfExtents;

    Box3D(const Vector3& center, const Vector3& halfExtents) : center(center), halfExtents(halfExtents) {}

    Vector3 getSupport(const Vector3& direction) const override {
        Vector3 result = center;
        result.x += (direction.x > 0) ? halfExtents.x : -halfExtents.x;
        result.y += (direction.y > 0) ? halfExtents.y : -halfExtents.y;
        result.z += (direction.z > 0) ? halfExtents.z : -halfExtents.z;
        return result;
    }
};

// Capsule Shape (Line segment with radius)
struct Capsule3D : public Shape3D {
    Vector3 startPoint;
    Vector3 endPoint;
    real radius;

    Capsule3D(const Vector3& start, const Vector3& end, real r) : startPoint(start), endPoint(end), radius(r) {}

    Vector3 getSupport(const Vector3& direction) const override {
        real dotStart = direction.dot(startPoint);
        real dotEnd = direction.dot(endPoint);

        Vector3 furthestPoint = (dotStart > dotEnd) ? startPoint : endPoint;
        Vector3 dir = direction.normalized();

        return furthestPoint + dir * radius;
    }
};

} // namespace Lite
