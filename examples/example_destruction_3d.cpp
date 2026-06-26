#include <iostream>
#include <vector>
#include <litephys/rigidbody3d/RigidBodyState3D.h>
#include <litephys/dynamics3d/Fracture3D.h>
#include <litephys/dynamics3d/Explosion3D.h>
#include <litephys/collisions3d/Shapes3D.h>
#include <litephys/collisions3d/BVH3D.h>
#include <litephys/math/Random.h>

using namespace Lite;

int main() {
    std::cout << "LitePhys - Dynamic Destruction & Explosions Test\n" << std::endl;

    Random rng(42);

    std::vector<RigidBodyState3D> bodies;
    std::vector<Box3D> shapes;

    // 1. Build a wall of 3x3 crates
    Vector3 extents(1.0f, 1.0f, 1.0f); // 2x2x2 boxes
    
    for (int y = 0; y < 3; ++y) {
        for (int x = -1; x <= 1; ++x) {
            RigidBodyState3D rb;
            rb.setMass(10.0f); // 10kg crates
            rb.setBoxInertia(extents);
            rb.position = Vector3((real)x * 2.05f, (real)y * 2.05f + 1.0f, 0.0f);
            
            Matrix3 rotMatrix = rb.orientation.toRotationMatrix();
            rb.inverseInertiaTensorWorld = rotMatrix * rb.inverseInertiaTensor * rotMatrix.transpose();

            bodies.push_back(rb);
            shapes.push_back(Box3D(rb.position, extents));
        }
    }

    std::cout << "Initial setup: " << bodies.size() << " unbroken crates." << std::endl;

    // 2. Build the BVH
    std::vector<std::pair<AABB3D, int>> aabbs;
    for (int i = 0; i < shapes.size(); ++i) {
        aabbs.push_back({AABB3D(shapes[i].center - shapes[i].halfExtents, shapes[i].center + shapes[i].halfExtents), i});
    }
    
    BVH3D bvh;
    bvh.buildStatic(aabbs);

    // 3. Simulate a lethal impact on the center crate (Index 4)
    int targetIndex = 4;
    std::cout << "\n[EVENT] High-velocity impact on Crate #" << targetIndex << "!" << std::endl;

    // Shatter it!
    std::vector<FracturePiece3D> shards = Fracture3D::shatterBox(bodies[targetIndex], shapes[targetIndex], rng);

    // Remove the old crate (Swap and pop for O(1) removal, though order matters for indices so we'll just overwrite it with the last, then add the rest)
    bodies[targetIndex] = bodies.back();
    shapes[targetIndex] = shapes.back();
    bodies.pop_back();
    shapes.pop_back();

    // Add the shards
    for (const auto& piece : shards) {
        bodies.push_back(piece.state);
        shapes.push_back(piece.shape);
    }

    std::cout << "Crate shattered into " << shards.size() << " pieces!" << std::endl;
    std::cout << "Total objects now: " << bodies.size() << std::endl;

    // Rebuild BVH since objects changed
    aabbs.clear();
    for (int i = 0; i < shapes.size(); ++i) {
        aabbs.push_back({AABB3D(shapes[i].center - shapes[i].halfExtents, shapes[i].center + shapes[i].halfExtents), i});
    }
    bvh.buildStatic(aabbs);

    // 4. Detonate an Explosion at the impact site!
    Vector3 blastCenter(0.0f, 3.0f, 0.0f); // Center of the wall
    real blastRadius = 5.0f;
    real blastForce = 500.0f; // Massive impulse

    std::cout << "\n[EVENT] Detonating Explosion! Radius: " << blastRadius << ", Force: " << blastForce << std::endl;
    
    Explosion3D::detonate(blastCenter, blastRadius, blastForce, bvh, bodies, rng);

    // 5. Step physics to observe the shrapnel flying
    std::cout << "\nSimulating 3 frames of shrapnel physics..." << std::endl;
    real dt = 1.0f / 60.0f;

    for (int frame = 0; frame < 3; ++frame) {
        std::cout << "\n--- Frame " << frame << " ---" << std::endl;
        
        for (size_t i = 0; i < bodies.size(); ++i) {
            auto& rb = bodies[i];
            
            // Integrate position
            rb.position += rb.velocity * dt;
            
            // Simple log
            if (i > 7) { // Only log the new shards
                std::cout << "Shard " << i << " | Pos: X=" << rb.position.x << " Y=" << rb.position.y << " Z=" << rb.position.z 
                          << " | Vel: " << rb.velocity.magnitude() << std::endl;
            }
        }
    }

    return 0;
}
