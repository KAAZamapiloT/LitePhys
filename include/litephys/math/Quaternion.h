#pragma once
#include "Precision.h"
#include "Vector3.h"
#include "Matrix3.h"
#include <cmath>

namespace Lite {

class Quaternion {
public:
    real r, i, j, k;

    Quaternion() : r(1), i(0), j(0), k(0) {}
    Quaternion(real r, real i, real j, real k) : r(r), i(i), j(j), k(k) {}

    void normalize() {
        real d = r*r + i*i + j*j + k*k;
        if (d == 0) {
            r = 1; return;
        }
        d = ((real)1.0) / std::sqrt(d);
        r *= d;
        i *= d;
        j *= d;
        k *= d;
    }

    Quaternion operator*(const Quaternion& multiplier) const {
        Quaternion q;
        q.r = r*multiplier.r - i*multiplier.i - j*multiplier.j - k*multiplier.k;
        q.i = r*multiplier.i + i*multiplier.r + j*multiplier.k - k*multiplier.j;
        q.j = r*multiplier.j + j*multiplier.r + k*multiplier.i - i*multiplier.k;
        q.k = r*multiplier.k + k*multiplier.r + i*multiplier.j - j*multiplier.i;
        return q;
    }

    void addScaledVector(const Vector3& vector, real scale) {
        Quaternion q(0,
            vector.x * scale,
            vector.y * scale,
            vector.z * scale);
        q = q * (*this);
        r += q.r * 0.5f;
        i += q.i * 0.5f;
        j += q.j * 0.5f;
        k += q.k * 0.5f;
    }

    // Convert to a rotation matrix to transform inertia tensors and local points to world space
    Matrix3 toRotationMatrix() const {
        Matrix3 m;
        m.data[0] = 1 - (2*j*j + 2*k*k);
        m.data[1] = 2*i*j - 2*k*r;
        m.data[2] = 2*i*k + 2*j*r;

        m.data[3] = 2*i*j + 2*k*r;
        m.data[4] = 1 - (2*i*i + 2*k*k);
        m.data[5] = 2*j*k - 2*i*r;

        m.data[6] = 2*i*k - 2*j*r;
        m.data[7] = 2*j*k + 2*i*r;
        m.data[8] = 1 - (2*i*i + 2*j*j);
        return m;
    }

    Vector3 rotate(const Vector3& vector) const {
        return toRotationMatrix() * vector;
    }
};

} // namespace Lite
