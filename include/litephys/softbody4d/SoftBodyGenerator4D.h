#pragma once
#include "SoftBodyState4D.h"
#include <cmath>

namespace Lite {

class SoftBodyGenerator4D {
public:
    static SoftBodyState4D createTesseractLattice(const Vector4& origin, real size, int resolution, real massPerParticle, real stiffness, real damping) {
        SoftBodyState4D softbody;
        
        real spacing = size / (resolution - 1);
        
        // Create Particles
        for (int w = 0; w < resolution; ++w) {
            for (int z = 0; z < resolution; ++z) {
                for (int y = 0; y < resolution; ++y) {
                    for (int x = 0; x < resolution; ++x) {
                        PhysicsState4D p;
                        p.position = origin + Vector4(x * spacing, y * spacing, z * spacing, w * spacing);
                        p.setMass(massPerParticle);
                        p.radius = spacing * 0.5f;
                        softbody.particles.push_back(p);
                    }
                }
            }
        }
        
        // Helper to get index
        auto getIndex = [&](int x, int y, int z, int w) {
            return x + y * resolution + z * resolution * resolution + w * resolution * resolution * resolution;
        };

        // Create Structural and Shear Springs (only forward + directions to avoid duplicates)
        for (int w = 0; w < resolution; ++w) {
            for (int z = 0; z < resolution; ++z) {
                for (int y = 0; y < resolution; ++y) {
                    for (int x = 0; x < resolution; ++x) {
                        int currentIndex = getIndex(x, y, z, w);
                        
                        // Just connect adjacent 4D neighbors for structural springs
                        int structuralOffsets[4][4] = {
                            {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}
                        };
                        
                        for (int i = 0; i < 4; ++i) {
                            int nx = x + structuralOffsets[i][0];
                            int ny = y + structuralOffsets[i][1];
                            int nz = z + structuralOffsets[i][2];
                            int nw = w + structuralOffsets[i][3];
                            
                            if (nx < resolution && ny < resolution && nz < resolution && nw < resolution) {
                                int neighborIndex = getIndex(nx, ny, nz, nw);
                                Vector4 diff = softbody.particles[currentIndex].position - softbody.particles[neighborIndex].position;
                                real restLength = diff.magnitude();
                                softbody.springs.push_back(SpringConstraint4D(currentIndex, neighborIndex, restLength, stiffness, damping));
                            }
                        }
                        
                        // Diagonal 2D plane shears (simplified set)
                        int shearOffsets[6][4] = {
                            {1, 1, 0, 0}, {1, 0, 1, 0}, {1, 0, 0, 1},
                            {0, 1, 1, 0}, {0, 1, 0, 1}, {0, 0, 1, 1}
                        };
                        
                        for (int i = 0; i < 6; ++i) {
                            int nx = x + shearOffsets[i][0];
                            int ny = y + shearOffsets[i][1];
                            int nz = z + shearOffsets[i][2];
                            int nw = w + shearOffsets[i][3];
                            
                            if (nx < resolution && ny < resolution && nz < resolution && nw < resolution) {
                                int neighborIndex = getIndex(nx, ny, nz, nw);
                                Vector4 diff = softbody.particles[currentIndex].position - softbody.particles[neighborIndex].position;
                                real restLength = diff.magnitude();
                                softbody.springs.push_back(SpringConstraint4D(currentIndex, neighborIndex, restLength, stiffness, damping));
                            }
                        }
                        
                        // We skip 3D and 4D interior shears here for simplicity to reduce spring count,
                        // but 4D lattice stability can be maintained with structural + 2D shears.
                    }
                }
            }
        }
        
        return softbody;
    }
};

} // namespace Lite
