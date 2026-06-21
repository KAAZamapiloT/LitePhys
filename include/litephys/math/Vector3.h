#pragma once
#include "Precision.h"

namespace Lite {

struct Vector3 {
    real x;
    real y;
    real z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(real x, real y, real z) : x(x), y(y), z(z) {}

    void invert() {
        x = -x;
        y = -y;
        z = -z;
    }

    real magnitude() const {
        return Math::real_sqrt(x * x + y * y + z * z);
    }

    real squareMagnitude() const {
        return x * x + y * y + z * z;
    }

    void normalize() {
        real l = magnitude();
        if (l > 0) {
            (*this) *= (static_cast<real>(1.0) / l);
        }
    }

    Vector3 normalized() const {
        Vector3 result = *this;
        result.normalize();
        return result;
    }

    // Fast normalization using approximate math
    void fastNormalize() {
        real invSqrt = Math::fast_inv_sqrt(x * x + y * y + z * z);
        x *= invSqrt;
        y *= invSqrt;
        z *= invSqrt;
    }

    Vector3 operator*(const real value) const {
        return Vector3(x * value, y * value, z * value);
    }

    void operator*=(const real value) {
        x *= value;
        y *= value;
        z *= value;
    }

    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    void operator+=(const Vector3& v) {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    void operator-=(const Vector3& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    void addScaledVector(const Vector3& vector, real scale) {
        x += vector.x * scale;
        y += vector.y * scale;
        z += vector.z * scale;
    }

    real dot(const Vector3& vector) const {
        return x * vector.x + y * vector.y + z * vector.z;
    }

    Vector3 cross(const Vector3& vector) const {
        return Vector3(
            y * vector.z - z * vector.y,
            z * vector.x - x * vector.z,
            x * vector.y - y * vector.x
        );
    }

    void clear() {
        x = y = z = 0;
    }
};

} // namespace Lite
