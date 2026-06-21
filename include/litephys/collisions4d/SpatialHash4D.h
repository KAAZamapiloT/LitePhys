#pragma once
#include "CollidableParticle4D.h"
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace Lite {

struct SpatialHash4D {
    real cellSize;
    std::unordered_map<uint64_t, std::vector<CollidableParticle4D*>> grid;

    SpatialHash4D(real cellSize) : cellSize(cellSize) {}

    uint64_t hashPosition(const Vector4& pos) const {
        int x = (int)Math::real_floor(pos.x / cellSize);
        int y = (int)Math::real_floor(pos.y / cellSize);
        int z = (int)Math::real_floor(pos.z / cellSize);
        int w = (int)Math::real_floor(pos.w / cellSize);

        uint64_t h1 = 0x8da6b343;
        uint64_t h2 = 0xd8163841;
        uint64_t h3 = 0xcb1ab31f;
        uint64_t h4 = 0x9e3779b9;

        uint64_t n = (x * h1) ^ (y * h2) ^ (z * h3) ^ (w * h4);
        return n;
    }

    void clear() {
        grid.clear();
    }

    void insert(CollidableParticle4D* particle) {
        uint64_t cellId = hashPosition(particle->getPosition());
        grid[cellId].push_back(particle);
    }

    std::vector<CollidableParticle4D*> getNearby(const Vector4& pos) {
        std::vector<CollidableParticle4D*> neighbors;
        
        int px = (int)Math::real_floor(pos.x / cellSize);
        int py = (int)Math::real_floor(pos.y / cellSize);
        int pz = (int)Math::real_floor(pos.z / cellSize);
        int pw = (int)Math::real_floor(pos.w / cellSize);

        for (int dw = -1; dw <= 1; ++dw) {
            for (int dz = -1; dz <= 1; ++dz) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        Vector4 cellPos((px + dx) * cellSize, (py + dy) * cellSize, (pz + dz) * cellSize, (pw + dw) * cellSize);
                        uint64_t cellId = hashPosition(cellPos);
                        
                        auto it = grid.find(cellId);
                        if (it != grid.end()) {
                            for (auto* p : it->second) {
                                neighbors.push_back(p);
                            }
                        }
                    }
                }
            }
        }
        return neighbors;
    }
};

} // namespace Lite
