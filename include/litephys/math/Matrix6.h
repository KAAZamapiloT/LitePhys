#pragma once
#include "Precision.h"
#include "Bivector4.h"

namespace Lite {

// 6x6 Matrix specifically to map 6D Bivectors to 6D Bivectors (Inertia Tensor in 4D)
class Matrix6 {
public:
    real data[36];

    Matrix6() {
        for (int i = 0; i < 36; ++i) data[i] = 0;
        data[0] = data[7] = data[14] = data[21] = data[28] = data[35] = 1.0f; // Identity by default
    }

    Bivector4 operator*(const Bivector4& b) const {
        return Bivector4(
            data[0]*b.xy + data[1]*b.xz + data[2]*b.xw + data[3]*b.yz + data[4]*b.yw + data[5]*b.zw,
            data[6]*b.xy + data[7]*b.xz + data[8]*b.xw + data[9]*b.yz + data[10]*b.yw + data[11]*b.zw,
            data[12]*b.xy + data[13]*b.xz + data[14]*b.xw + data[15]*b.yz + data[16]*b.yw + data[17]*b.zw,
            data[18]*b.xy + data[19]*b.xz + data[20]*b.xw + data[21]*b.yz + data[22]*b.yw + data[23]*b.zw,
            data[24]*b.xy + data[25]*b.xz + data[26]*b.xw + data[27]*b.yz + data[28]*b.yw + data[29]*b.zw,
            data[30]*b.xy + data[31]*b.xz + data[32]*b.xw + data[33]*b.yz + data[34]*b.yw + data[35]*b.zw
        );
    }
    
    // We can manually set the diagonal for isotropic inertia (e.g. hyperspheres)
    void setDiagonal(real d) {
        for (int i = 0; i < 36; ++i) data[i] = 0;
        data[0] = data[7] = data[14] = data[21] = data[28] = data[35] = d;
    }
};

} // namespace Lite
