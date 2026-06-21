#pragma once
#include "Precision.h"
#include "Vector3.h"

namespace Lite {

class Matrix3 {
public:
    real data[9];

    Matrix3() {
        for (int i = 0; i < 9; ++i) data[i] = 0;
        data[0] = data[4] = data[8] = 1.0f; // Identity by default
    }

    Matrix3(real m0, real m1, real m2,
            real m3, real m4, real m5,
            real m6, real m7, real m8) {
        data[0] = m0; data[1] = m1; data[2] = m2;
        data[3] = m3; data[4] = m4; data[5] = m5;
        data[6] = m6; data[7] = m7; data[8] = m8;
    }

    Vector3 operator*(const Vector3& vector) const {
        return Vector3(
            vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
            vector.x * data[3] + vector.y * data[4] + vector.z * data[5],
            vector.x * data[6] + vector.y * data[7] + vector.z * data[8]
        );
    }

    Matrix3 operator*(const Matrix3& o) const {
        return Matrix3(
            data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6],
            data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7],
            data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8],

            data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6],
            data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7],
            data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8],

            data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6],
            data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7],
            data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8]
        );
    }

    Matrix3 transpose() const {
        return Matrix3(
            data[0], data[3], data[6],
            data[1], data[4], data[7],
            data[2], data[5], data[8]
        );
    }

    void setInverse(const Matrix3& m) {
        real t4 = m.data[0]*m.data[4];
        real t6 = m.data[0]*m.data[5];
        real t8 = m.data[1]*m.data[3];
        real t10 = m.data[2]*m.data[3];
        real t12 = m.data[1]*m.data[6];
        real t14 = m.data[2]*m.data[6];

        real t16 = (t4*m.data[8] - t6*m.data[7] - t8*m.data[8] +
                    t10*m.data[7] + t12*m.data[5] - t14*m.data[4]);

        if (t16 == (real)0.0) return;
        real t17 = 1 / t16;

        data[0] = (m.data[4]*m.data[8] - m.data[5]*m.data[7]) * t17;
        data[1] = -(m.data[1]*m.data[8] - m.data[2]*m.data[7]) * t17;
        data[2] = (m.data[1]*m.data[5] - m.data[2]*m.data[4]) * t17;
        data[3] = -(m.data[3]*m.data[8] - m.data[5]*m.data[6]) * t17;
        data[4] = (m.data[0]*m.data[8] - t14) * t17;
        data[5] = -(t6 - t10) * t17;
        data[6] = (m.data[3]*m.data[7] - m.data[4]*m.data[6]) * t17;
        data[7] = -(m.data[0]*m.data[7] - t12) * t17;
        data[8] = (t4 - t8) * t17;
    }

    Matrix3 inverse() const {
        Matrix3 result;
        result.setInverse(*this);
        return result;
    }
};

} // namespace Lite
