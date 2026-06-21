#pragma once
#include "Vector4.h"

namespace Lite {

// Simple 4x4 Matrix for 4D space transformations
struct Matrix4 {
    real data[16];

    Matrix4() {
        for (int i=0; i<16; ++i) data[i] = 0;
        data[0] = data[5] = data[10] = data[15] = 1.0f; // Identity
    }

    Vector4 transform(const Vector4& vector) const {
        return Vector4(
            vector.x * data[0] + vector.y * data[4] + vector.z * data[8] + vector.w * data[12],
            vector.x * data[1] + vector.y * data[5] + vector.z * data[9] + vector.w * data[13],
            vector.x * data[2] + vector.y * data[6] + vector.z * data[10] + vector.w * data[14],
            vector.x * data[3] + vector.y * data[7] + vector.z * data[11] + vector.w * data[15]
        );
    }

    Matrix4 operator+(const Matrix4& o) const {
        Matrix4 m;
        for (int i=0; i<16; ++i) m.data[i] = data[i] + o.data[i];
        return m;
    }

    Matrix4 operator*(real s) const {
        Matrix4 m;
        for (int i=0; i<16; ++i) m.data[i] = data[i] * s;
        return m;
    }

    Matrix4 operator*(const Matrix4& o) const {
        Matrix4 m;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m.data[i*4 + j] = 
                    data[i*4 + 0] * o.data[0*4 + j] +
                    data[i*4 + 1] * o.data[1*4 + j] +
                    data[i*4 + 2] * o.data[2*4 + j] +
                    data[i*4 + 3] * o.data[3*4 + j];
            }
        }
        return m;
    }

    Matrix4 transpose() const {
        Matrix4 m;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m.data[i*4 + j] = data[j*4 + i];
            }
        }
        return m;
    }

    // Returns the column vectors
    Vector4 getCol(int col) const {
        return Vector4(data[col], data[col+4], data[col+8], data[col+12]);
    }

    void setCol(int col, const Vector4& v) {
        data[col] = v.x;
        data[col+4] = v.y;
        data[col+8] = v.z;
        data[col+12] = v.w;
    }

    // Gram-Schmidt orthogonalization to prevent rotation matrix drift
    void orthogonalize() {
        Vector4 c0 = getCol(0);
        Vector4 c1 = getCol(1);
        Vector4 c2 = getCol(2);
        Vector4 c3 = getCol(3);

        c0.normalize();

        c1 = c1 - c0 * c1.dot(c0);
        c1.normalize();

        c2 = c2 - c0 * c2.dot(c0) - c1 * c2.dot(c1);
        c2.normalize();

        c3 = c3 - c0 * c3.dot(c0) - c1 * c3.dot(c1) - c2 * c3.dot(c2);
        c3.normalize();

        setCol(0, c0);
        setCol(1, c1);
        setCol(2, c2);
        setCol(3, c3);
    }

    // Generate a rotation matrix for a specific 4D plane
    static Matrix4 makeRotationXY(real angle) {
        Matrix4 m;
        real c = Math::real_cos(angle);
        real s = Math::real_sin(angle);
        m.data[0] = c; m.data[4] = -s;
        m.data[1] = s; m.data[5] = c;
        return m;
    }

    static Matrix4 makeRotationXZ(real angle) {
        Matrix4 m;
        real c = Math::real_cos(angle);
        real s = Math::real_sin(angle);
        m.data[0] = c; m.data[8] = -s;
        m.data[2] = s; m.data[10] = c;
        return m;
    }

    static Matrix4 makeRotationXW(real angle) {
        Matrix4 m;
        real c = Math::real_cos(angle);
        real s = Math::real_sin(angle);
        m.data[0] = c; m.data[12] = -s;
        m.data[3] = s; m.data[15] = c;
        return m;
    }

    static Matrix4 makeRotationYZ(real angle) {
        Matrix4 m;
        real c = Math::real_cos(angle);
        real s = Math::real_sin(angle);
        m.data[5] = c; m.data[9] = -s;
        m.data[6] = s; m.data[10] = c;
        return m;
    }

    static Matrix4 makeRotationYW(real angle) {
        Matrix4 m;
        real c = Math::real_cos(angle);
        real s = Math::real_sin(angle);
        m.data[5] = c; m.data[13] = -s;
        m.data[7] = s; m.data[15] = c;
        return m;
    }

    static Matrix4 makeRotationZW(real angle) {
        Matrix4 m;
        real c = Math::real_cos(angle);
        real s = Math::real_sin(angle);
        m.data[10] = c; m.data[14] = -s;
        m.data[11] = s; m.data[15] = c;
        return m;
    }
};

} // namespace Lite
