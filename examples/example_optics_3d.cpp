#include <iostream>
#include <litephys/LiteOptics3D.h>

using namespace Lite;

int main() {
    std::cout << "LitePhys - 3D Optics Example" << std::endl;

    OpticsWorld3D optics;
    
    // Add a glass sphere
    Sphere3D sphere(Vector3(0, 0, 5), 2.0f);
    sphere.refractiveIndex = 1.5f; // Glass
    optics.addShape(&sphere);

    // Add a mirror plane behind the sphere
    Plane3D mirror(Vector3(0, 0, -1), 10.0f); // Normal pointing back, at z=10
    optics.addShape(&mirror);

    // Shoot a ray slightly offset from the center
    Ray3D ray(Vector3(0.5f, 0, 0), Vector3(0, 0, 1));
    
    std::cout << "Shooting Ray from (" << ray.origin.x << ", " << ray.origin.y << ", " << ray.origin.z << ") "
              << "Direction: (" << ray.direction.x << ", " << ray.direction.y << ", " << ray.direction.z << ")" << std::endl;

    auto path = optics.traceRay(ray, 3); // Trace up to 3 bounces

    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << "Bounce " << (i+1) << ": Hit at (" 
                  << path[i].point.x << ", " << path[i].point.y << ", " << path[i].point.z 
                  << ") -> New Direction: (" 
                  << path[i].direction.x << ", " << path[i].direction.y << ", " << path[i].direction.z 
                  << ") [" << (path[i].isRefraction ? "Refraction" : "Reflection") << "]" << std::endl;
    }

    return 0;
}
