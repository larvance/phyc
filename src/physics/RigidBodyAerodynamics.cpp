#include "physics/RigidBody.hpp"
#include "physics/World.hpp"

using namespace PhyC;

// approximation, assumes the following:
// inviscid flow
// no separation
// no circulation
// no pressure recovery
// no Reynolds dependence
// no Mach effects
// flat plate behavior everywhere
double RigidBody::solveLocalPressure(const vec3& normal, const vec3& relativeVelocity, const FluidProperties& air) {
    double v_n = relativeVelocity.dot(normal);

    return 0.5 * air.density * v_n * abs(v_n); // dynamic pressure
}

// approximation
// not reynolds-dependent
// not laminar <-> turbulent aware
vec3 RigidBody::solveLocalShearStress(const vec3& normal, const vec3& relativeVelocity, const FluidProperties& air) {
    vec3 v_t = relativeVelocity - relativeVelocity.dot(normal) * normal;

    double speed_t = v_t.norm();

    if (speed_t < 1e-6) return vec3::Zero();

    constexpr double Cf = 0.005;

    vec3 shear = -0.5 * air.density * Cf * speed_t * v_t;

    return shear;
}

void RigidBody::computeAeroSurfaceForces(FTState& state, const FluidProperties& air) const {
    mat3 R = state.rot.toRotationMatrix();

    for (const auto& elem : surface.elements) {
        vec3 r_body = elem.offset - pStableCenterOfMass;
        vec3 r = R * r_body;

        vec3 n = R * elem.normal;

        vec3 surfaceVelocity = state.vel + state.angVel.cross(r);

        // using freestream (V_inf) air velocity only.
        // each surface element sees undisturbed air.
        // this does NOT solve the flow field:
        // no induced velocity,
        // no wake
        // no downwash
        // no surface-to-surface interaction.
        vec3 relativeAirVelocity = air.vel - surfaceVelocity;

        double pressure = solveLocalPressure(n, relativeAirVelocity, air);
        vec3 shearStress = solveLocalShearStress(n, relativeAirVelocity, air);

        vec3 dF = (pressure * n + shearStress) * elem.area;

        state.applyForceDist(dF, r);
    }
}

// not exact, but efficient and accurate
void RigidBody::computeAeroDampingTorque(FTState& state, const FluidProperties& air) const {
    vec3 omega_body = state.rot.inverse() * state.angVel;

    double rho = air.density;
    double V = (state.vel - world->sampleAirProperties(state.pos).vel).norm();
    if (V < 1e-6) return;

    double S = 0.42; // m^2, area of the wings when looked from above
    double b = 1.3;  // m, wingspan, not one corner to another, the straight line on the wings
    double c = S / b;

    double Lp = 0.5 * rho * V * S * b * b * Clp;
    double Mq = 0.5 * rho * V * S * c * c * Cmq;
    double Nr = 0.5 * rho * V * S * b * b * Cnr;

    vec3 torque_body(
        Lp * omega_body.x(),
        Mq * omega_body.y(),
        Nr * omega_body.z()
    );

    state.applyTorque(state.rot * torque_body);
}
