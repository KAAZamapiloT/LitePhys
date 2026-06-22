#pragma once
#include "Ray4D.h"
#include <algorithm>

namespace Lite {

struct AABB4D {
    Vector4 min;
    Vector4 max;

    AABB4D() {
        min = Vector4(99999.0f, 99999.0f, 99999.0f, 99999.0f);
        max = Vector4(-99999.0f, -99999.0f, -99999.0f, -99999.0f);
    }
    
    AABB4D(const Vector4& min, const Vector4& max) : min(min), max(max) {}

    bool overlaps(const AABB4D& other) const {
        if (max.x < other.min.x || min.x > other.max.x) return false;
        if (max.y < other.min.y || min.y > other.max.y) return false;
        if (max.z < other.min.z || min.z > other.max.z) return false;
        if (max.w < other.min.w || min.w > other.max.w) return false;
        return true;
    }

    void expand(const Vector4& point) {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        min.z = std::min(min.z, point.z);
        min.w = std::min(min.w, point.w);
        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
        max.z = std::max(max.z, point.z);
        max.w = std::max(max.w, point.w);
    }

    void expand(const AABB4D& other) {
        expand(other.min);
        expand(other.max);
    }

    real getSurfaceArea() const {
        // Approximate "Surface Volume" for 4D splitting heuristic
        Vector4 d = max - min;
        return 2.0f * (d.x*d.y*d.z + d.x*d.y*d.w + d.x*d.z*d.w + d.y*d.z*d.w);
    }

    Vector4 getCenter() const {
        return (min + max) * 0.5f;
    }

    // Slab method for 4D Ray-AABB intersection
    bool intersectRay(const Ray4D& ray, real& tOut, real tMax = 99999.0f) const {
        real tMin = 0.0f;
        real tMaxBox = tMax;

        for (int i = 0; i < 4; ++i) {
            real origin = (i == 0) ? ray.origin.x : ((i == 1) ? ray.origin.y : ((i == 2) ? ray.origin.z : ray.origin.w));
            real dir = (i == 0) ? ray.direction.x : ((i == 1) ? ray.direction.y : ((i == 2) ? ray.direction.z : ray.direction.w));
            real bMin = (i == 0) ? min.x : ((i == 1) ? min.y : ((i == 2) ? min.z : min.w));
            real bMax = (i == 0) ? max.x : ((i == 1) ? max.y : ((i == 2) ? max.z : max.w));

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
