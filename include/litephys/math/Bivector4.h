#pragma once
#include "Precision.h"
#include "Vector4.h"
#include "Matrix4.h"

namespace Lite {

class Bivector4 {
public:
    // 6 components representing the 6 planes of rotation in 4D
    real xy, xz, xw, yz, yw, zw;

    Bivector4() : xy(0), xz(0), xw(0), yz(0), yw(0), zw(0) {}
    Bivector4(real xy, real xz, real xw, real yz, real yw, real zw)
        : xy(xy), xz(xz), xw(xw), yz(yz), yw(yw), zw(zw) {}

    Bivector4 operator+(const Bivector4& v) const {
        return Bivector4(xy + v.xy, xz + v.xz, xw + v.xw, yz + v.yz, yw + v.yw, zw + v.zw);
    }

    Bivector4 operator-(const Bivector4& v) const {
        return Bivector4(xy - v.xy, xz - v.xz, xw - v.xw, yz - v.yz, yw - v.yw, zw - v.zw);
    }

    Bivector4 operator*(real s) const {
        return Bivector4(xy * s, xz * s, xw * s, yz * s, yw * s, zw * s);
    }

    // Dot product between two bivectors
    real dot(const Bivector4& v) const {
        return xy * v.xy + xz * v.xz + xw * v.xw + yz * v.yz + yw * v.yw + zw * v.zw;
    }

    // Wedge product of two vectors produces a Bivector
    // Used for calculating Torque = r ^ F
    static Bivector4 wedge(const Vector4& a, const Vector4& b) {
        return Bivector4(
            a.x * b.y - a.y * b.x,
            a.x * b.z - a.z * b.x,
            a.x * b.w - a.w * b.x,
            a.y * b.z - a.z * b.y,
            a.y * b.w - a.w * b.y,
            a.z * b.w - a.w * b.z
        );
    }

    // Contraction of a Bivector with a Vector produces a Vector
    // Used for calculating linear velocity from angular velocity: v = w * r
    Vector4 contract(const Vector4& v) const {
        // Equivalent to multiplying the vector by the skew-symmetric matrix representation of the bivector
        return Vector4(
            -xy * v.y - xz * v.z - xw * v.w,
             xy * v.x - yz * v.z - yw * v.w,
             xz * v.x + yz * v.y - zw * v.w,
             xw * v.x + yw * v.y + zw * v.z
        );
    }

    // Convert to a 4x4 skew-symmetric matrix for updating orientation matrices
    Matrix4 toSkewSymmetricMatrix() const {
        Matrix4 m;
        m.data[0] = 0;   m.data[1] = -xy; m.data[2] = -xz; m.data[3] = -xw;
        m.data[4] = xy;  m.data[5] = 0;   m.data[6] = -yz; m.data[7] = -yw;
        m.data[8] = xz;  m.data[9] = yz;  m.data[10] = 0;  m.data[11] = -zw;
        m.data[12] = xw; m.data[13] = yw; m.data[14] = zw; m.data[15] = 0;
        return m;
    }
};

} // namespace Lite
