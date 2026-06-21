#pragma once
#include "../math/Vector3.h"
#include <assert.h>

namespace Lite {

class Particle {
protected:
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;
    
    // Amount of damping applied to linear motion.
    // Damping is required to remove energy added through numerical instability in the integrator.
    real damping;

    // Inverse mass is more useful than mass for integration, as mass=0 is infinite mass (immovable).
    real inverseMass;

    // Accumulated force to be applied at the next integration step.
    Vector3 forceAccum;

public:
    Particle() : damping(static_cast<real>(0.999)), inverseMass(static_cast<real>(1.0)) {
        position.clear();
        velocity.clear();
        acceleration.clear();
        forceAccum.clear();
    }

    void integrate(real duration) {
        if (inverseMass <= 0.0f) return; // Infinite mass, doesn't move
        assert(duration > 0.0);

        // Update linear position.
        position.addScaledVector(velocity, duration);

        // Work out the acceleration from the force
        Vector3 resultingAcc = acceleration;
        resultingAcc.addScaledVector(forceAccum, inverseMass);

        // Update linear velocity from the acceleration.
        velocity.addScaledVector(resultingAcc, duration);

        // Impose drag.
        velocity *= Math::real_pow(damping, duration);

        // Clear the forces.
        clearAccumulator();
    }

    // Set mass (calculates inverse mass automatically)
    void setMass(real mass) {
        assert(mass != 0);
        inverseMass = static_cast<real>(1.0) / mass;
    }

    real getMass() const {
        if (inverseMass == 0) {
            return std::numeric_limits<real>::max();
        } else {
            return static_cast<real>(1.0) / inverseMass;
        }
    }

    void setInverseMass(real inverseMass) {
        this->inverseMass = inverseMass;
    }

    real getInverseMass() const {
        return inverseMass;
    }

    bool hasFiniteMass() const {
        return inverseMass >= 0.0f;
    }

    void setDamping(real damping) {
        this->damping = damping;
    }

    real getDamping() const {
        return damping;
    }

    void setPosition(const Vector3& position) {
        this->position = position;
    }

    void setPosition(real x, real y, real z) {
        position.x = x;
        position.y = y;
        position.z = z;
    }

    Vector3 getPosition() const {
        return position;
    }

    void setVelocity(const Vector3& velocity) {
        this->velocity = velocity;
    }

    void setVelocity(real x, real y, real z) {
        velocity.x = x;
        velocity.y = y;
        velocity.z = z;
    }

    Vector3 getVelocity() const {
        return velocity;
    }

    void setAcceleration(const Vector3& acceleration) {
        this->acceleration = acceleration;
    }

    void setAcceleration(real x, real y, real z) {
        acceleration.x = x;
        acceleration.y = y;
        acceleration.z = z;
    }

    Vector3 getAcceleration() const {
        return acceleration;
    }

    // Clear accumulator
    void clearAccumulator() {
        forceAccum.clear();
    }

    // Add force to the accumulator
    void addForce(const Vector3& force) {
        forceAccum += force;
    }
};

} // namespace Lite
