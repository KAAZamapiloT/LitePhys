#include <iostream>
#include <litephys/LiteOptics4D.h>

using namespace Lite;

int main() {
    std::cout << "LitePhys - 4D Optics Example" << std::endl;

    OpticsSolver4D optics;
    
    // Add a glass hyper-sphere
    HyperSphere4D hypersphere(Vector4(0, 0, 0, 5), 2.0f);
    hypersphere.refractiveIndex = 1.33f; // Water
    optics.addShape(&hypersphere);

    // Shoot a ray slightly offset from the center in X, heading straight into W
    Ray4D ray(Vector4(1.0f, 0, 0, 0), Vector4(0, 0, 0, 1));
    
    std::cout << "Shooting Ray from (" << ray.origin.x << ", " << ray.origin.y << ", " << ray.origin.z << ", " << ray.origin.w << ") "
              << "Direction: (" << ray.direction.x << ", " << ray.direction.y << ", " << ray.direction.z << ", " << ray.direction.w << ")" << std::endl;

    auto path = optics.traceRay(ray, 2);

    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << "Bounce " << (i+1) << ": Hit at (" 
                  << path[i].point.x << ", " << path[i].point.y << ", " << path[i].point.z << ", " << path[i].point.w
                  << ") -> New Direction: (" 
                  << path[i].direction.x << ", " << path[i].direction.y << ", " << path[i].direction.z << ", " << path[i].direction.w
                  << ") [" << (path[i].isRefraction ? "Refraction" : "Reflection") << "]" << std::endl;
    }

    return 0;
}
