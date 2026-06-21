#include <iostream>
#include <vector>
#include <litephys/LitePhys3D.h>

using namespace Lite;

// Mock Game Engine Component System
struct PhysicsComponent {
    PhysicsState3D state;
};

int main() {
    std::cout << "LitePhys - Stateless ECS Gravity Example" << std::endl;

    // The game engine owns the entities and components
    std::vector<PhysicsComponent> components(1);
    
    // Initialize our component
    components[0].state.position = Vector3(0, 10, 0);
    components[0].state.setMass(2.0f);

    Vector3 gravity(0, -9.81f, 0);
    const real dt = 0.1f;

    // Mock Game Loop
    for (int i = 0; i <= 20; ++i) {
        std::cout << "Time: " << (i * dt) << "s - ";
        std::cout << "Position: (" << components[0].state.position.x << ", " 
                                   << components[0].state.position.y << ", " 
                                   << components[0].state.position.z << ")" << std::endl;

        // The game engine explicitly calls the stateless physics providers
        for (auto& comp : components) {
            ForceSolver3D::applyGravity(comp.state, gravity);
            Integrator3D::integrate(comp.state, dt);

            // Simple floor bounce inside the engine's loop
            if (comp.state.position.y < 0.0f) {
                comp.state.position.y = 0.0f;
                comp.state.velocity.y = -comp.state.velocity.y * 0.8f;
            }
        }
    }

    return 0;
}
