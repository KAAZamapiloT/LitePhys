#pragma once
#include "Precision.h"

namespace Lite {

struct Vector4 {
    real x;
    real y;
    real z;
    real w;

    Vector4() : x(0), y(0), z(0), w(0) {}
    Vector4(real x, real y, real z, real w) : x(x), y(y), z(z), w(w) {}

    void invert() {
        x = -x; y = -y; z = -z; w = -w;
    }

    real magnitude() const {
        return Math::real_sqrt(x*x + y*y + z*z + w*w);
    }

    real squareMagnitude() const {
        return x*x + y*y + z*z + w*w;
    }

    void normalize() {
        real l = magnitude();
        if (l > 0) {
            (*this) *= (static_cast<real>(1.0) / l);
        }
    }

    Vector4 normalized() const {
        Vector4 result = *this;
        result.normalize();
        return result;
    }

    Vector4 operator*(const real value) const {
        return Vector4(x*value, y*value, z*value, w*value);
    }

    void operator*=(const real value) {
        x *= value; y *= value; z *= value; w *= value;
    }

    Vector4 operator+(const Vector4& v) const {
        return Vector4(x+v.x, y+v.y, z+v.z, w+v.w);
    }

    void operator+=(const Vector4& v) {
        x += v.x; y += v.y; z += v.z; w += v.w;
    }

    Vector4 operator-(const Vector4& v) const {
        return Vector4(x-v.x, y-v.y, z-v.z, w-v.w);
    }

    void operator-=(const Vector4& v) {
        x -= v.x; y -= v.y; z -= v.z; w -= v.w;
    }

    void addScaledVector(const Vector4& vector, real scale) {
        x += vector.x * scale;
        y += vector.y * scale;
        z += vector.z * scale;
        w += vector.w * scale;
    }

    real dot(const Vector4& vector) const {
        return x*vector.x + y*vector.y + z*vector.z + w*vector.w;
    }

    void clear() {
        x = y = z = w = 0;
    }
};

} // namespace Lite
