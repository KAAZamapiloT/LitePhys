#pragma once
#include "Vector4.h"

namespace Lite {

// Simple 4x4 Matrix for 4D space transformations
struct alignas(16) Matrix4 {
    union {
#if LITEPHYS_USE_SSE
        __m128 rows[4];
#endif
        real data[16];
    };

    Matrix4() {
#if LITEPHYS_USE_SSE
        rows[0] = _mm_set_ps(0, 0, 0, 1);
        rows[1] = _mm_set_ps(0, 0, 1, 0);
        rows[2] = _mm_set_ps(0, 1, 0, 0);
        rows[3] = _mm_set_ps(1, 0, 0, 0);
#else
        for (int i=0; i<16; ++i) data[i] = 0;
        data[0] = data[5] = data[10] = data[15] = 1.0f; // Identity
#endif
    }

    Vector4 transform(const Vector4& vector) const {
#if LITEPHYS_USE_SSE
        __m128 v = vector.simd;
        // x * col0 + y * col1 + z * col2 + w * col3
        // To do this easily, we can broadcast vector components and multiply by columns.
        // Wait, rows are stored, so doing dot products with rows is faster.
        __m128 dot0 = _mm_dp_ps(rows[0], v, 0xFF);
        __m128 dot1 = _mm_dp_ps(rows[1], v, 0xFF);
        __m128 dot2 = _mm_dp_ps(rows[2], v, 0xFF);
        __m128 dot3 = _mm_dp_ps(rows[3], v, 0xFF);
        
        // Now combine them. dot0 has result in all 4 floats.
        // We want (dot0.x, dot1.x, dot2.x, dot3.x)
        __m128 res01 = _mm_movelh_ps(dot0, dot1); // (dot0.x, dot0.y, dot1.x, dot1.y) -> actually both are same
        __m128 res23 = _mm_movelh_ps(dot2, dot3); 
        __m128 final = _mm_shuffle_ps(res01, res23, _MM_SHUFFLE(2, 0, 2, 0));
        return Vector4(final);
#else
        return Vector4(
            vector.x * data[0] + vector.y * data[1] + vector.z * data[2] + vector.w * data[3],
            vector.x * data[4] + vector.y * data[5] + vector.z * data[6] + vector.w * data[7],
            vector.x * data[8] + vector.y * data[9] + vector.z * data[10] + vector.w * data[11],
            vector.x * data[12] + vector.y * data[13] + vector.z * data[14] + vector.w * data[15]
        );
#endif
    }

    Matrix4 operator+(const Matrix4& o) const {
        Matrix4 m;
#if LITEPHYS_USE_SSE
        m.rows[0] = _mm_add_ps(rows[0], o.rows[0]);
        m.rows[1] = _mm_add_ps(rows[1], o.rows[1]);
        m.rows[2] = _mm_add_ps(rows[2], o.rows[2]);
        m.rows[3] = _mm_add_ps(rows[3], o.rows[3]);
#else
        for (int i=0; i<16; ++i) m.data[i] = data[i] + o.data[i];
#endif
        return m;
    }

    Matrix4 operator*(real s) const {
        Matrix4 m;
#if LITEPHYS_USE_SSE
        __m128 scalar = _mm_set1_ps(s);
        m.rows[0] = _mm_mul_ps(rows[0], scalar);
        m.rows[1] = _mm_mul_ps(rows[1], scalar);
        m.rows[2] = _mm_mul_ps(rows[2], scalar);
        m.rows[3] = _mm_mul_ps(rows[3], scalar);
#else
        for (int i=0; i<16; ++i) m.data[i] = data[i] * s;
#endif
        return m;
    }

    Matrix4 operator*(const Matrix4& o) const {
        Matrix4 m;
#if LITEPHYS_USE_SSE
        for (int i = 0; i < 4; ++i) {
            __m128 row = rows[i];
            __m128 v0 = _mm_mul_ps(_mm_shuffle_ps(row, row, _MM_SHUFFLE(0, 0, 0, 0)), o.rows[0]);
            __m128 v1 = _mm_mul_ps(_mm_shuffle_ps(row, row, _MM_SHUFFLE(1, 1, 1, 1)), o.rows[1]);
            __m128 v2 = _mm_mul_ps(_mm_shuffle_ps(row, row, _MM_SHUFFLE(2, 2, 2, 2)), o.rows[2]);
            __m128 v3 = _mm_mul_ps(_mm_shuffle_ps(row, row, _MM_SHUFFLE(3, 3, 3, 3)), o.rows[3]);
            m.rows[i] = _mm_add_ps(_mm_add_ps(v0, v1), _mm_add_ps(v2, v3));
        }
#else
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m.data[i*4 + j] = 
                    data[i*4 + 0] * o.data[0*4 + j] +
                    data[i*4 + 1] * o.data[1*4 + j] +
                    data[i*4 + 2] * o.data[2*4 + j] +
                    data[i*4 + 3] * o.data[3*4 + j];
            }
        }
#endif
        return m;
    }

    Matrix4 transpose() const {
        Matrix4 m;
#if LITEPHYS_USE_SSE
        __m128 tmp0 = _mm_unpacklo_ps(rows[0], rows[1]);
        __m128 tmp2 = _mm_unpacklo_ps(rows[2], rows[3]);
        __m128 tmp1 = _mm_unpackhi_ps(rows[0], rows[1]);
        __m128 tmp3 = _mm_unpackhi_ps(rows[2], rows[3]);

        m.rows[0] = _mm_movelh_ps(tmp0, tmp2);
        m.rows[1] = _mm_movehl_ps(tmp2, tmp0);
        m.rows[2] = _mm_movelh_ps(tmp1, tmp3);
        m.rows[3] = _mm_movehl_ps(tmp3, tmp1);
#else
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m.data[i*4 + j] = data[j*4 + i];
            }
        }
#endif
        return m;
    }

    Vector4 getCol(int col) const {
#if LITEPHYS_USE_SSE
        Matrix4 t = transpose();
        return Vector4(t.rows[col]);
#else
        return Vector4(data[col], data[col+4], data[col+8], data[col+12]);
#endif
    }

    void setCol(int col, const Vector4& v) {
#if LITEPHYS_USE_SSE
        Matrix4 t = transpose();
        t.rows[col] = v.simd;
        *this = t.transpose();
#else
        data[col] = v.x;
        data[col+4] = v.y;
        data[col+8] = v.z;
        data[col+12] = v.w;
#endif
    }

    // Gram-Schmidt orthogonalization
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
