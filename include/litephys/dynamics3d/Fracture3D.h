#pragma once
#include <vector>
#include "../rigidbody3d/RigidBodyState3D.h"
#include "../collisions3d/Shapes3D.h"
#include "../math/Random.h"

namespace Lite {

struct FracturePiece3D {
    RigidBodyState3D state;
    Box3D shape;
};

class Fracture3D {
public:
    // Shatters a single Box into 8 smaller boxes (2x2x2)
    // Conserves mass and momentum.
    static std::vector<FracturePiece3D> shatterBox(
        const RigidBodyState3D& parentState,
        const Box3D& parentShape,
        Random& rng)
    {
        std::vector<FracturePiece3D> shards;
        shards.reserve(8);

        // Each child gets 1/8th of the parent's mass
        real childMass = parentState.mass * 0.125f;
        Vector3 childExtents = parentShape.halfExtents * 0.5f;

        // Iterate through the 8 octants
        for (int i = 0; i < 8; ++i) {
            real signX = (i & 1) ? 1.0f : -1.0f;
            real signY = (i & 2) ? 1.0f : -1.0f;
            real signZ = (i & 4) ? 1.0f : -1.0f;

            // The local offset from the center of the parent
            Vector3 localOffset(
                signX * childExtents.x,
                signY * childExtents.y,
                signZ * childExtents.z
            );

            // Add some random jitter to the local offset to prevent perfect grid stacks
            localOffset.x += rng.range(-0.1f, 0.1f) * childExtents.x;
            localOffset.y += rng.range(-0.1f, 0.1f) * childExtents.y;
            localOffset.z += rng.range(-0.1f, 0.1f) * childExtents.z;

            // Rotate offset by parent's orientation to get world offset
            Vector3 worldOffset = parentState.orientation.rotate(localOffset);

            FracturePiece3D piece = {
                RigidBodyState3D(),
                Box3D(Vector3(0,0,0), childExtents) // Center updated in state later
            };

            // Setup state
            piece.state.position = parentState.position + worldOffset;
            piece.shape.center = piece.state.position;
            piece.state.orientation = parentState.orientation;
            
            // Conserve linear momentum: V_point = V_center + (AngularVelocity x Offset)
            piece.state.velocity = parentState.velocity + parentState.angularVelocity.cross(worldOffset);
            
            // Inherit angular momentum
            piece.state.angularVelocity = parentState.angularVelocity;

            // Add a tiny bit of explosive radial jitter so they separate immediately rather than clipping
            Vector3 separationJitter = worldOffset.normalized() * rng.range(0.5f, 2.0f);
            piece.state.velocity += separationJitter;
            
            // Small rotational tumble jitter
            piece.state.angularVelocity += rng.randomDirection() * rng.range(0.5f, 1.5f);

            // Setup inertia
            piece.state.setMass(childMass);
            piece.state.setBoxInertia(childExtents);
            
            // Calculate inverse inertia tensor world
            Matrix3 rotMatrix = piece.state.orientation.toRotationMatrix();
            piece.state.inverseInertiaTensorWorld = rotMatrix * piece.state.inverseInertiaTensor * rotMatrix.transpose();

            shards.push_back(piece);
        }

        return shards;
    }
};

} // namespace Lite
