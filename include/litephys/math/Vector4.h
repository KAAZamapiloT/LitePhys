#pragma once
#include "Precision.h"

// Check for SSE support
#if defined(__SSE__) || (_M_IX86_FP > 0) || defined(_M_AMD64) || defined(_M_X64)
    #define LITEPHYS_USE_SSE 1
    #include <immintrin.h>
#else
    #define LITEPHYS_USE_SSE 0
#endif

namespace Lite {

struct alignas(16) Vector4 {
    union {
#if LITEPHYS_USE_SSE
        __m128 simd;
#endif
        struct { real x, y, z, w; };
        real data[4];
    };

    Vector4() {
#if LITEPHYS_USE_SSE
        simd = _mm_setzero_ps();
#else
        x = 0; y = 0; z = 0; w = 0;
#endif
    }

    Vector4(real x, real y, real z, real w) {
#if LITEPHYS_USE_SSE
        simd = _mm_set_ps(w, z, y, x); // _mm_set_ps takes arguments in reverse order (w, z, y, x)
#else
        this->x = x; this->y = y; this->z = z; this->w = w;
#endif
    }

#if LITEPHYS_USE_SSE
    Vector4(__m128 v) : simd(v) {}
#endif

    void invert() {
#if LITEPHYS_USE_SSE
        __m128 zero = _mm_setzero_ps();
        simd = _mm_sub_ps(zero, simd);
#else
        x = -x; y = -y; z = -z; w = -w;
#endif
    }

    real squareMagnitude() const {
#if LITEPHYS_USE_SSE
        __m128 mul = _mm_mul_ps(simd, simd);
        // Horizontal add
        __m128 hadd1 = _mm_hadd_ps(mul, mul);
        __m128 hadd2 = _mm_hadd_ps(hadd1, hadd1);
        return _mm_cvtss_f32(hadd2);
#else
        return x*x + y*y + z*z + w*w;
#endif
    }

    real magnitude() const {
        return Math::real_sqrt(squareMagnitude());
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
#if LITEPHYS_USE_SSE
        return Vector4(_mm_mul_ps(simd, _mm_set1_ps(value)));
#else
        return Vector4(x*value, y*value, z*value, w*value);
#endif
    }

    void operator*=(const real value) {
#if LITEPHYS_USE_SSE
        simd = _mm_mul_ps(simd, _mm_set1_ps(value));
#else
        x *= value; y *= value; z *= value; w *= value;
#endif
    }

    Vector4 operator+(const Vector4& v) const {
#if LITEPHYS_USE_SSE
        return Vector4(_mm_add_ps(simd, v.simd));
#else
        return Vector4(x+v.x, y+v.y, z+v.z, w+v.w);
#endif
    }

    void operator+=(const Vector4& v) {
#if LITEPHYS_USE_SSE
        simd = _mm_add_ps(simd, v.simd);
#else
        x += v.x; y += v.y; z += v.z; w += v.w;
#endif
    }

    Vector4 operator-(const Vector4& v) const {
#if LITEPHYS_USE_SSE
        return Vector4(_mm_sub_ps(simd, v.simd));
#else
        return Vector4(x-v.x, y-v.y, z-v.z, w-v.w);
#endif
    }

    void operator-=(const Vector4& v) {
#if LITEPHYS_USE_SSE
        simd = _mm_sub_ps(simd, v.simd);
#else
        x -= v.x; y -= v.y; z -= v.z; w -= v.w;
#endif
    }

    void addScaledVector(const Vector4& vector, real scale) {
#if LITEPHYS_USE_SSE
        simd = _mm_add_ps(simd, _mm_mul_ps(vector.simd, _mm_set1_ps(scale)));
#else
        x += vector.x * scale;
        y += vector.y * scale;
        z += vector.z * scale;
        w += vector.w * scale;
#endif
    }

    real dot(const Vector4& vector) const {
#if LITEPHYS_USE_SSE
        __m128 mul = _mm_mul_ps(simd, vector.simd);
        __m128 hadd1 = _mm_hadd_ps(mul, mul);
        __m128 hadd2 = _mm_hadd_ps(hadd1, hadd1);
        return _mm_cvtss_f32(hadd2);
#else
        return x*vector.x + y*vector.y + z*vector.z + w*vector.w;
#endif
    }

    void clear() {
#if LITEPHYS_USE_SSE
        simd = _mm_setzero_ps();
#else
        x = y = z = w = 0;
#endif
    }
};

} // namespace Lite
