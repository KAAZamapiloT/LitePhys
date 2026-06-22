#include <iostream>
#include <litephys/collisions4d/Shapes4D.h>
#include <litephys/collisions4d/GJK4D.h>
#include <litephys/collisions4d/EPA4D.h>

using namespace Lite;

void testCollision4D(const Shape4D& a, const Shape4D& b) {
    Simplex4D simplex;
    bool intersects = GJK4D::checkCollision(a, b, simplex);
    
    if (intersects) {
        std::cout << "COLLISION DETECTED!" << std::endl;
        std::pair<Vector4, real> penetration = EPA4D::getPenetration(a, b, simplex);
        std::cout << "  Normal: (" << penetration.first.x << ", " << penetration.first.y << ", " << penetration.first.z << ", " << penetration.first.w << ")" << std::endl;
        std::cout << "  Depth:  " << penetration.second << std::endl;
    } else {
        std::cout << "No Collision." << std::endl;
    }
}

int main() {
    std::cout << "LitePhys - GJK/EPA 4D Collision Test" << std::endl;

    Hypercube4D cubeA(Vector4(0, 0, 0, 0), Vector4(1, 1, 1, 1)); // 2x2x2x2 Hypercube at origin
    Sphere4D sphereB(Vector4(2.5f, 0, 0, 0), 1.0f);    // 4D Sphere at X=2.5, radius=1.0. Misses by 0.5

    std::cout << "\nTest 1: Hypercube at Origin vs 4D Sphere at X=2.5 - Should NOT collide" << std::endl;
    testCollision4D(cubeA, sphereB);

    sphereB.center = Vector4(1.5f, 0, 0, 0); // Intersects hypercube by 0.5 along X axis
    std::cout << "\nTest 2: Hypercube at Origin vs 4D Sphere at X=1.5 - Should collide" << std::endl;
    testCollision4D(cubeA, sphereB);

    Spherocylinder4D capsule(Vector4(0, 5, 0, 0), Vector4(0, 10, 0, 0), 1.0f); // 4D Capsule standing on Y=4 (5-1)
    Hypercube4D floor(Vector4(0, 0, 0, 0), Vector4(10, 0.5f, 10, 10)); // Floor surface at Y=0.5

    std::cout << "\nTest 3: 4D Capsule at Y=5 vs Floor at Y=0.5 - Should NOT collide" << std::endl;
    testCollision4D(capsule, floor);

    capsule.startPoint.y = 1.0f; // Bottom of capsule (Y=1), radius 1 -> touches Y=0
    capsule.endPoint.y = 6.0f;
    std::cout << "\nTest 4: 4D Capsule shifted down vs Floor - Should collide" << std::endl;
    testCollision4D(capsule, floor);

    return 0;
}
