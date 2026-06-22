#include <iostream>
#include <litephys/collisions3d/Shapes3D.h>
#include <litephys/collisions3d/GJK3D.h>
#include <litephys/collisions3d/EPA3D.h>

using namespace Lite;

void testCollision(const Shape3D& a, const Shape3D& b) {
    Simplex3D simplex;
    bool intersects = GJK3D::checkCollision(a, b, simplex);
    
    if (intersects) {
        std::cout << "COLLISION DETECTED!" << std::endl;
        std::pair<Vector3, real> penetration = EPA3D::getPenetration(a, b, simplex);
        std::cout << "  Normal: (" << penetration.first.x << ", " << penetration.first.y << ", " << penetration.first.z << ")" << std::endl;
        std::cout << "  Depth:  " << penetration.second << std::endl;
    } else {
        std::cout << "No Collision." << std::endl;
    }
}

int main() {
    std::cout << "LitePhys - GJK/EPA 3D Collision Test" << std::endl;

    Box3D boxA(Vector3(0, 0, 0), Vector3(1, 1, 1)); // 2x2x2 Box at origin
    Sphere3D sphereB(Vector3(2.5f, 0, 0), 1.0f);    // Sphere at X=2.5, radius=1.0. Misses box by 0.5

    std::cout << "\nTest 1: Box at (0,0,0) vs Sphere at (2.5, 0, 0) - Should NOT collide" << std::endl;
    testCollision(boxA, sphereB);

    sphereB.center = Vector3(1.5f, 0, 0); // Intersects box by 0.5 along X axis
    std::cout << "\nTest 2: Box at (0,0,0) vs Sphere at (1.5, 0, 0) - Should collide (Depth 0.5)" << std::endl;
    testCollision(boxA, sphereB);

    Capsule3D capsule(Vector3(0, 5, 0), Vector3(0, 10, 0), 1.0f); // Capsule standing on Y=4 (5-1)
    Box3D floor(Vector3(0, 0, 0), Vector3(10, 0.5f, 10)); // Floor surface at Y=0.5

    std::cout << "\nTest 3: Capsule at Y=5 vs Floor at Y=0.5 - Should NOT collide" << std::endl;
    testCollision(capsule, floor);

    capsule.startPoint.y = 1.0f; // Bottom of capsule (Y=1), radius 1 -> touches Y=0
    capsule.endPoint.y = 6.0f;
    std::cout << "\nTest 4: Capsule shifted down vs Floor - Should collide (Depth 0.5)" << std::endl;
    testCollision(capsule, floor);

    return 0;
}
