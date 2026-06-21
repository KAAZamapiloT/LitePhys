#include <iostream>
#include <vector>
#include <litephys/LitePhys4D.h>

using namespace Lite;

// Mock Game Engine Component System
struct PhysicsComponent4D {
    PhysicsState4D state;
};

int main() {
    std::cout << "LitePhys - Stateless ECS 4D Gravity Example" << std::endl;

    std::vector<PhysicsComponent4D> components(1);
    
    components[0].state.position = Vector4(0, 10, 0, 5);
    components[0].state.setMass(2.0f);

    Vector4 gravity(0, -9.81f, 0, -4.0f);
    
    // Rotate initial velocity
    Matrix4 rotXW = Matrix4::makeRotationXW(Lite::PI / 4.0f); 
    Vector4 initialVelocity(2.0f, 0.0f, 0.0f, 0.0f);
    components[0].state.velocity = rotXW.transform(initialVelocity);

    const real dt = 0.1f;

    // Mock Game Loop
    for (int i = 0; i <= 20; ++i) {
        std::cout << "Time: " << (i * dt) << "s - ";
        std::cout << "Position: (" << components[0].state.position.x << ", " 
                                   << components[0].state.position.y << ", " 
                                   << components[0].state.position.z << ", "
                                   << components[0].state.position.w << ")" << std::endl;

        for (auto& comp : components) {
            ForceSolver4D::applyGravity(comp.state, gravity);
            Integrator4D::integrate(comp.state, dt);

            if (comp.state.position.y < 0.0f) {
                comp.state.position.y = 0.0f;
                comp.state.velocity.y = -comp.state.velocity.y * 0.8f;
            }
            if (comp.state.position.w < 0.0f) {
                comp.state.position.w = 0.0f;
                comp.state.velocity.w = -comp.state.velocity.w * 0.8f;
            }
        }
    }

    return 0;
}
