#include "RigidBody.hpp"
#include "World.hpp"

using namespace PhyC;

std::atomic<unsigned long long> PhyC::next_id{0};

void RigidBody::precomputeMass() {
    if (isStatic) {
        pStableCenterOfMass.setZero();
        pMass = 0.0;
        pInvMass = 0.0;
        return;
    }
    pStableCenterOfMass.setZero();
    pMass = 0.0;

    for (const auto& s : surface) {
        double m = s.mass();
        pStableCenterOfMass += m * s.offset;
        pMass += m;
    }

    for (const auto& m : extraMasses) {
        pStableCenterOfMass += m.mass * m.position;
        pMass += m.mass;
    }

    if (pMass > 1e-9) {
        pStableCenterOfMass /= pMass;
    } else {
        pMass = 1.0;
        std::cerr << "WARNING: Rigid Body mass was calculated to be less than 1e-9, defaulting to 1kg" << std::endl;
    }

    pInvMass = 1.0 / pMass;
}

void RigidBody::precomputeInertia() {
    mat3 I = mat3::Zero();

    for (const auto& s : surface) {
        double m = s.mass();

        vec3 v0 = (s.offset + s.corners.col(0)) - pStableCenterOfMass;
        vec3 v1 = (s.offset + s.corners.col(1)) - pStableCenterOfMass;
        vec3 v2 = (s.offset + s.corners.col(2)) - pStableCenterOfMass;

        vec3 center = (v0 + v1 + v2) / 3.0;

        mat3 C_plate = (m / 12.0) * (
            (v0 * v0.transpose()) +
            (v1 * v1.transpose()) +
            (v2 * v2.transpose()) +
            (9.0 * center * center.transpose())
        );

        double trace_C = C_plate(0, 0) + C_plate(1, 1) + C_plate(2, 2);
        mat3 I_plate = (trace_C * mat3::Identity()) - C_plate;

        if (s.thickness > 1e-9) {
            double t2_12 = (s.thickness * s.thickness) / 12.0;
            mat3 nnt = s.normal * s.normal.transpose();
            mat3 I_thick = m * t2_12 * (mat3::Identity() - nnt);
            I_plate += I_thick;
        }

        I += I_plate;
    }

    mat3 I3 = mat3::Identity();
    for (const auto& m : extraMasses) {
        vec3 r = m.position - pStableCenterOfMass;
        I += m.mass * ((r.squaredNorm() * I3) - (r * r.transpose()));
    }

    double minInertia = 1e-6;
    for (int i = 0; i < 3; ++i)
        if (I(i, i) < minInertia) I(i, i) = minInertia;

    if (isStatic) {
        pBodyInertia.setZero();
        pInvBodyInertia.setZero();
        invWorldInertia.setZero();
        worldInertia.setZero();
    } else {
        pBodyInertia = I;
        pInvBodyInertia = I.inverse();
    }

    precomputed = true;
}

void RigidBody::computeWorldInertia() {
    if (isStatic) {
        worldInertia.setZero();
        invWorldInertia.setZero();
        return;
    }
    mat3 R = curState.rot.toRotationMatrix();
    worldInertia = R * pBodyInertia * R.transpose();
    invWorldInertia = R * pInvBodyInertia * R.transpose();
}

void RigidBody::evaluateRK4(Derivative& out, const State& initial, const Derivative& d, double dt) const {
    FTState state;
    state.pos = initial.pos + d.dPos * dt;
    state.vel = initial.vel + d.dVel * dt;
    state.angVel = initial.angVel + d.dAngVel * dt;

    // Quaternion integration: q_new = q + (q_dot * dt)
    state.rot = initial.rot;
    state.rot.coeffs() += d.dRot.coeffs() * dt;
    state.rot.normalize();

    computeForcesAt(state, dt);

    out.dPos = state.vel;
    out.dVel = state.force() * pInvMass;

    // Angular Acceleration: I^-1 * (Tau - w x (I * w))
    mat3 R = state.rot.toRotationMatrix();
    mat3 I_inv = R * pInvBodyInertia * R.transpose();
    mat3 I = R * pBodyInertia * R.transpose();
    out.dAngVel = I_inv * (state.torque() - state.angVel.cross(I * state.angVel));

    // Quaternion Derivative: 0.5 * w * q
    quaternion w_quat(0, state.angVel.x(), state.angVel.y(), state.angVel.z());
    out.dRot = w_quat * state.rot;
    out.dRot.coeffs() *= 0.5;
}

void RigidBody::integrateRK4(double dt) {
    State initial{curState};

    Derivative a, b, c, d;

    // 4 k values:
    evaluateRK4(a, initial, Derivative(), 0.0f);
    evaluateRK4(b, initial, a, dt * 0.5f);
    evaluateRK4(c, initial, b, dt * 0.5f);
    evaluateRK4(d, initial, c, dt);

    // Final Integration: 1/6 * (a + 2b + 2c + d)
    vec3 dPosDt = (1.0 / 6.0) * (a.dPos + 2.0 * b.dPos + 2.0 * c.dPos + d.dPos);
    vec3 dVelDt = (1.0 / 6.0) * (a.dVel + 2.0 * b.dVel + 2.0 * c.dVel + d.dVel);
    vec3 dAngVelDt = (1.0 / 6.0) * (a.dAngVel + 2.0 * b.dAngVel + 2.0 * c.dAngVel + d.dAngVel);

    quaternion dRotDt;
    dRotDt.coeffs() = (1.0 / 6.0) * (a.dRot.coeffs() + 2.0 * b.dRot.coeffs() + 2.0 * c.dRot.coeffs() + d.dRot.coeffs());

    curState.pos += dPosDt * dt;
    curState.vel += dVelDt * dt;
    curState.angVel += dAngVelDt * dt;

    curState.rot.coeffs() += dRotDt.coeffs() * dt;
    curState.rot.normalize();
}

void RigidBody::integrateEuler(double dt) {
    FTState state{curState};
    computeForcesAt(state, dt);

    vec3 acc = state.force() * pInvMass;
    curState.pos += (curState.vel + 0.5 * dt * acc) * dt;
    curState.vel += acc * dt;

    vec3 angAcc = invWorldInertia * (state.torque() - curState.angVel.cross(worldInertia * curState.angVel));
    vec3 angVelHalf = curState.angVel + 0.5 * dt * angAcc;

    double angle = angVelHalf.norm() * dt;
    if (angle > 1e-12) {
        vec3 axis = angVelHalf.normalized();
        quaternion dq(Eigen::AngleAxisd(angle, axis));
        curState.rot = dq * curState.rot;
    }
    curState.rot.normalize();

    curState.angVel += angAcc * dt;
}

volatile bool useRK4 = true;

void RigidBody::update(double dt) {
    if (isStatic) return;

    if (useRK4) integrateRK4(dt);
    else integrateEuler(dt);

    alignCollidingBodies();
}

void RigidBody::computeForcesAt(FTState& state, double dt) const {
    computeGravityForce(state);

    // AirProperties air = world->sampleAirProperties(state.pos);

    // computeAeroSurfaceForces(state, air);

    // computeAeroDampingTorque(state, air);
}

void RigidBody::computeGravityForce(FTState& state) const {
    state.applyForce(pMass * world->gravity);
}

void RigidBody::remove() {
    if (world) world->removeRigidBody(*this);
}
