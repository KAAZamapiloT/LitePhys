#pragma once
#include "../math/Vector4.h"
#include <assert.h>

namespace Lite {

class Particle4D {
protected:
    Vector4 position;
    Vector4 velocity;
    Vector4 acceleration;
    
    real damping;
    real inverseMass;
    Vector4 forceAccum;

public:
    Particle4D() : damping(static_cast<real>(0.999)), inverseMass(static_cast<real>(1.0)) {
        position.clear();
        velocity.clear();
        acceleration.clear();
        forceAccum.clear();
    }

    void integrate(real duration) {
        if (inverseMass <= 0.0f) return;
        assert(duration > 0.0);

        position.addScaledVector(velocity, duration);

        Vector4 resultingAcc = acceleration;
        resultingAcc.addScaledVector(forceAccum, inverseMass);

        velocity.addScaledVector(resultingAcc, duration);
        velocity *= Math::real_pow(damping, duration);

        clearAccumulator();
    }

    void setMass(real mass) {
        assert(mass != 0);
        inverseMass = static_cast<real>(1.0) / mass;
    }

    real getMass() const {
        if (inverseMass == 0) return std::numeric_limits<real>::max();
        return static_cast<real>(1.0) / inverseMass;
    }

    bool hasFiniteMass() const { return inverseMass >= 0.0f; }
    
    void setDamping(real damping) { this->damping = damping; }
    real getDamping() const { return damping; }

    void setPosition(const Vector4& position) { this->position = position; }
    void setPosition(real x, real y, real z, real w) {
        position.x = x; position.y = y; position.z = z; position.w = w;
    }
    Vector4 getPosition() const { return position; }

    void setVelocity(const Vector4& velocity) { this->velocity = velocity; }
    void setVelocity(real x, real y, real z, real w) {
        velocity.x = x; velocity.y = y; velocity.z = z; velocity.w = w;
    }
    Vector4 getVelocity() const { return velocity; }

    void setAcceleration(const Vector4& acceleration) { this->acceleration = acceleration; }
    void setAcceleration(real x, real y, real z, real w) {
        acceleration.x = x; acceleration.y = y; acceleration.z = z; acceleration.w = w;
    }
    Vector4 getAcceleration() const { return acceleration; }

    void clearAccumulator() { forceAccum.clear(); }
    void addForce(const Vector4& force) { forceAccum += force; }
};

} // namespace Lite
