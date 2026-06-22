#pragma once
#include "Ray3D.h"
#include <algorithm>

namespace Lite {

struct AABB3D {
    Vector3 min;
    Vector3 max;

    AABB3D() {
        min = Vector3(99999.0f, 99999.0f, 99999.0f);
        max = Vector3(-99999.0f, -99999.0f, -99999.0f);
    }
    
    AABB3D(const Vector3& min, const Vector3& max) : min(min), max(max) {}

    bool overlaps(const AABB3D& other) const {
        if (max.x < other.min.x || min.x > other.max.x) return false;
        if (max.y < other.min.y || min.y > other.max.y) return false;
        if (max.z < other.min.z || min.z > other.max.z) return false;
        return true;
    }

    void expand(const Vector3& point) {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);
        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
    }

    void expand(const AABB3D& other) {
        expand(other.min);
        expand(other.max);
    }

    real getSurfaceArea() const {
        Vector3 d = max - min;
        return 2.0f * (d.x*d.y + d.y*d.z + d.x*d.z);
    }

    Vector3 getCenter() const {
        return (min + max) * 0.5f;
    }

    // Slab method for Ray-AABB intersection
    bool intersectRay(const Ray3D& ray, real& tOut, real tMax = 99999.0f) const {
        real tMin = 0.0f;
        real tMaxBox = tMax;

        for (int i = 0; i < 3; ++i) {
            real origin = (i == 0) ? ray.origin.x : ((i == 1) ? ray.origin.y : ray.origin.z);
            real dir = (i == 0) ? ray.direction.x : ((i == 1) ? ray.direction.y : ray.direction.z);
            real bMin = (i == 0) ? min.x : ((i == 1) ? min.y : min.z);
            real bMax = (i == 0) ? max.x : ((i == 1) ? max.y : max.z);

            if (std::abs(dir) < 0.000001f) {
                if (origin < bMin || origin > bMax) return false;
            } else {
                real t1 = (bMin - origin) / dir;
                real t2 = (bMax - origin) / dir;
                
                if (t1 > t2) std::swap(t1, t2);
                
                tMin = std::max(tMin, t1);
                tMaxBox = std::min(tMaxBox, t2);
                
                if (tMin > tMaxBox) return false;
            }
        }

        tOut = tMin;
        return true;
    }
};

} // namespace Lite
