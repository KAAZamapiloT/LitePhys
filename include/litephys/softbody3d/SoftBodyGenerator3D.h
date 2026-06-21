#pragma once
#include "SoftBodyState3D.h"
#include <cmath>

namespace Lite {

class SoftBodyGenerator3D {
public:
    static SoftBodyState3D createCubeLattice(const Vector3& origin, real size, int resolution, real massPerParticle, real stiffness, real damping) {
        SoftBodyState3D softbody;
        
        real spacing = size / (resolution - 1);
        
        // Create Particles
        for (int z = 0; z < resolution; ++z) {
            for (int y = 0; y < resolution; ++y) {
                for (int x = 0; x < resolution; ++x) {
                    PhysicsState3D p;
                    p.position = origin + Vector3(x * spacing, y * spacing, z * spacing);
                    p.setMass(massPerParticle);
                    p.radius = spacing * 0.5f; // For particle collisions
                    softbody.particles.push_back(p);
                }
            }
        }
        
        // Helper to get index
        auto getIndex = [&](int x, int y, int z) {
            return x + y * resolution + z * resolution * resolution;
        };

        // Create Structural and Shear Springs
        for (int z = 0; z < resolution; ++z) {
            for (int y = 0; y < resolution; ++y) {
                for (int x = 0; x < resolution; ++x) {
                    int currentIndex = getIndex(x, y, z);
                    
                    // Link to neighbors (forward directions only to avoid duplicates)
                    int offsets[13][3] = {
                        {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, // Structural
                        {1, 1, 0}, {1, -1, 0}, {1, 0, 1}, {1, 0, -1}, {0, 1, 1}, {0, 1, -1}, // Face Shear
                        {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1} // Internal Shear
                    };
                    
                    for (int i = 0; i < 13; ++i) {
                        int nx = x + offsets[i][0];
                        int ny = y + offsets[i][1];
                        int nz = z + offsets[i][2];
                        
                        if (nx >= 0 && nx < resolution && ny >= 0 && ny < resolution && nz >= 0 && nz < resolution) {
                            int neighborIndex = getIndex(nx, ny, nz);
                            Vector3 diff = softbody.particles[currentIndex].position - softbody.particles[neighborIndex].position;
                            real restLength = diff.magnitude();
                            softbody.springs.push_back(SpringConstraint3D(currentIndex, neighborIndex, restLength, stiffness, damping));
                        }
                    }
                }
            }
        }
        
        return softbody;
    }
};

} // namespace Lite
