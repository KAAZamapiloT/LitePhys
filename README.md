# LitePhys

![LitePhys Logo](docs/logo.png)

A feather-light, header-only C++ physics library designed to be easily hooked into existing game engines. 
LitePhys is intentionally kept minimal—it is a *helper*, not a full game engine. It provides the core mathematics and integration steps for physics simulations, leaving collision resolution and rendering to the user.

## Features
- **Header-Only**: Just drop the `include/litephys` folder into your project.
- **Dependency-Free**: Includes custom math implementations.
- **True Modularity**: Only include what you need. 3D and 4D physics are completely separate modules.
- **Light Physics (Optics)**: Full raycasting system for both 3D and 4D environments with refraction (Snell's Law) and reflection calculations!
- **Tweakable Engine Hooks**: Easily implement custom constraints, bounds checking, or collisions via `postUpdateHook`.

## Modular Includes
LitePhys is split into independent modules. Use umbrella headers to pull in the features you want without bloating your project:

```cpp
// For standard 3D Particle physics:
#include <litephys/LitePhys3D.h>

// For 3D Raycasting and Optics:
#include <litephys/LiteOptics3D.h>

// For 4D Physics / Hyper-dimensional logic:
#include <litephys/LitePhys4D.h>
#include <litephys/LiteOptics4D.h>
```

## Precision Configuration
By default, LitePhys uses `float` for all calculations. You can change this by defining macros before including `LitePhys.h`:

```cpp
// For high precision mode (uses long double):
#define LITE_PHYSICS_DOUBLE_PRECISION

// For fast approximate math (e.g. Quake 3 fast inverse square root):
#define LITE_PHYSICS_APPROX_MATH

#include <litephys/LitePhys.h>
```

## Quick Start Example
Here's a minimal example of simulating a particle falling under gravity and bouncing on a floor.

```cpp
#include <iostream>
#include <litephys/LitePhys.h>

using namespace Lite;

// Tweakable hook to handle a simple floor collision
void checkBounds(Particle* p, real duration) {
    if (p->getPosition().y < 0.0f) {
        Vector3 pos = p->getPosition();
        pos.y = 0.0f;
        p->setPosition(pos);

        Vector3 vel = p->getVelocity();
        vel.y = -vel.y * 0.8f; // 0.8 is the bounciness
        p->setVelocity(vel);
    }
}

int main() {
    ParticleWorld world;
    world.setPostUpdateHook(checkBounds);

    Particle p1;
    p1.setPosition(0, 10, 0); // Start at height 10
    p1.setMass(2.0f);
    world.addParticle(&p1);

    ParticleGravity gravity(Vector3(0, -9.81f, 0));
    world.getForceRegistry().add(&p1, &gravity);

    // Simulate loop
    const real dt = 0.016f; // ~60fps
    for (int i = 0; i < 600; ++i) {
        world.startFrame();
        world.runPhysics(dt);
        // ... render p1.getPosition() ...
    }

    return 0;
}
```

## Building the Examples
```bash
mkdir build
cd build
cmake ..
cmake --build .
./example_gravity
```
