#include <iostream>
#include <litephys/collisions3d/Shapes3D.h>
#include <litephys/collisions3d/CCD3D.h>

using namespace Lite;

int main() {
    std::cout << "LitePhys - Continuous Collision Detection (CCD) Test\n" << std::endl;

    // A fast moving bullet (sphere of radius 0.1)
    Sphere3D bullet(Vector3(-10, 0, 0), 0.1f);
    Vector3 bulletVel(20.0f, 0, 0); // Moves 20 units per frame!

    // A thin wall (box half extents 0.5)
    Box3D wall(Vector3(5, 0, 0), Vector3(0.5f, 10.0f, 10.0f));
    Vector3 wallVel(0, 0, 0); // Static wall

    std::cout << "Bullet Start: X=" << bullet.center.x << " | Velocity=" << bulletVel.x << "/frame" << std::endl;
    std::cout << "Wall Center: X=" << wall.center.x << " | Thickness=1.0" << std::endl;

    // Standard Discrete Collision check (will fail because bullet jumps over the wall)
    Sphere3D bulletNext = bullet;
    bulletNext.center += bulletVel;
    
    Simplex3D simplex;
    bool discreteHit = GJK3D::checkCollision(bulletNext, wall, simplex);
    std::cout << "\n[Discrete Collision Check] Hit? " << (discreteHit ? "YES" : "NO (Tunneled through!)") << std::endl;

    // CCD Check
    real toi = 0.0f;
    bool ccdHit = CCD3D::checkCollisionCCD(bullet, bulletVel, wall, wallVel, toi);
    
    std::cout << "\n[CCD Check] Hit? " << (ccdHit ? "YES" : "NO") << std::endl;
    if (ccdHit) {
        std::cout << "  Time of Impact: " << toi << std::endl;
        Vector3 impactPos = bullet.center + bulletVel * toi;
        std::cout << "  Impact Position: X=" << impactPos.x << " (Correctly stopped at the wall boundary)" << std::endl;
    }

    return 0;
}
