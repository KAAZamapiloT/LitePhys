<div align="center">
  <img src="assets/logo.png" alt="LitePhys Logo" width="300">

  <h1>LitePhys</h1>
  <p>A lightweight, stateless, Entity-Component-System (ECS) friendly physics and optics provider for C++ game engines. Supports seamless 3D and 4D mathematical physics.</p>
</div>

---

## 🌟 Philosophy: A Provider, Not An Engine
Most physics libraries force you to adopt their `World` managers, their memory allocators, and their update loops. 

**LitePhys is different.** It is designed as a *Stateless Physics Provider*. It provides pure Data Structures (`structs`) and Static Functions (`solvers`). Your Game Engine retains 100% control over the game loop, the ECS architecture, and memory management.

## 🚀 Key Features
- **Header-Only**: Just drop the `include/` directory into your project. Zero external dependencies.
- **Stateless Integration**: Explicitly call semi-implicit Euler integrators during your engine's tick.
- **Rigid Body Dynamics**: Full 3D and 4D rigid body components featuring Quaternions/Orientation Matrices, Inertia Tensors, and Manifold-based impulse collisions supporting off-center torque!
- **Soft Body Dynamics**: Mass-Spring lattice models for 3D and 4D jello-like soft bodies with structural and shear constraints.
- **Multithreading (C++23)**: `std::execution::par_unseq` parallel solvers. Successfully stress-tested at real-time speeds with **100,000 bodies** via optional lock-free collision synchronization.
- **Fast Broad-Phase Collisions**: O(N) Spatial Hashing grids capable of culling thousands of objects instantly.
- **Stable Narrow-Phase Collisions**: Sphere-to-Sphere Velocity Impulse and Penalty Spring resolution.
- **N-Dimensional Math**: Full Vector/Matrix math libraries for both 3D and 4D.
- **Geometric Optics**: Snell's law refraction and raytracing through 3D/4D Hyper-Spheres and Hyper-Planes.

## 📦 Architecture Overview
You can cherry-pick exactly the math and logic you need using umbrella headers:
- `#include <litephys/LitePhys3D.h>` - Math, Integration, and Force Solvers (Gravity, Springs)
- `#include <litephys/LiteRigidBody3D.h>` - Rigid Body State, Integration, and Manifold Collisions
- `#include <litephys/LiteCollisions3D.h>` - Spatial Hashing and Particle Collision Resolution
- `#include <litephys/LiteOptics3D.h>` - Raytracing, Reflections, and Refractions

*(All of the above have a `4D` equivalent! Just replace `3D` with `4D` in the include paths).*

## 🛠️ Quick Start (ECS Example)

Here is a simplified example of how you can use LitePhys directly within your own engine's update loop without surrendering control:

```cpp
#include <vector>
#include <litephys/LitePhys3D.h>
#include <litephys/LiteCollisions3D.h>

// Your game engine's custom ECS component
struct MyEntity {
    Lite::PhysicsState3D physics;
    // ... custom graphics and game logic data
};

void EngineUpdate(std::vector<MyEntity>& entities, float dt) {
    Lite::SpatialHash3D spatialHash(2.0f);
    Lite::Vector3 gravity(0, -9.8f, 0);

    // 1. Integration Phase (Your Engine controls the loop!)
    for (auto& entity : entities) {
        Lite::ForceSolver3D::applyGravity(entity.physics, gravity);
        Lite::Integrator3D::integrate(entity.physics, dt);
    }

    // 2. Collision Phase
    // Extract state references for the stateless solver
    std::vector<Lite::PhysicsState3D> states; 
    for (auto& e : entities) states.push_back(e.physics);
    
    // Resolve collisions via Spatial Hashing
    Lite::CollisionResolver3D::resolve(states, spatialHash, Lite::CollisionMode::FAST_IMPULSE);
    
    // Sync back to ECS
    for (size_t i = 0; i < entities.size(); ++i) {
        entities[i].physics = states[i];
    }
}
```

## 🧪 Examples
Check out the `examples/` directory for full implementations:
- `example_gravity` & `example_gravity_4d`: Demonstrates basic force application and integration.
- `example_collisions_3d` & `example_collisions_4d`: Demonstrates resolving thousands of high-speed collisions using the Spatial Hash.
- `example_rigidbody_3d` & `example_rigidbody_4d`: Demonstrates full rigid body mechanics with off-center impulses generating angular spin.
- `example_softbody_3d` & `example_softbody_4d`: Demonstrates dropping Mass-Spring jello cubes/tesseracts onto a floor.
- `example_optics_3d` & `example_optics_4d`: Demonstrates tracing rays through glass and water volumes with physically accurate refractions.

### How to Build and Run Examples
To build all of the examples, run the following commands from the project root:

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

After the build completes, the executables will be located in the `build/Debug/` (or `build/Release/` depending on your generator) directory.

To run an example on Windows:
```bash
.\Debug\example_rigidbody_3d.exe
```

To run an example on Linux/macOS:
```bash
./example_rigidbody_3d
```

## 📝 License
MIT License. Free for use in any personal or commercial game engine.
