#include "physics/joints/FixedJoint.hpp"

void PhyC::FixedJoint::solve(double dt) {
    if (!parent || !child) return;
    if (parent->isStatic && child->isStatic) return;

    constexpr double beta = 0.2;

    vec3 rA = parent->curState.rot * parentAnchor;
    vec3 rB = child->curState.rot * childAnchor;

    vec3 pA = parent->curState.pos + rA;
    vec3 pB = child->curState.pos + rB;

    vec3 vA = parent->curState.vel + parent->curState.angVel.cross(rA);
    vec3 vB = child->curState.vel + child->curState.angVel.cross(rB);

    vec3 Cdot = vB - vA;

    vec3 C = pB - pA;
    vec3 bias = (beta / dt) * C;

    double invMassA = parent->isStatic ? 0.0 : parent->pInvMass;
    double invMassB = child->isStatic ? 0.0 : child->pInvMass;

    mat3 invIA = parent->isStatic ? mat3::Zero() : parent->invWorldInertia;
    mat3 invIB = child->isStatic ? mat3::Zero() : child->invWorldInertia;

    mat3 K = mat3::Identity() * (invMassA + invMassB)
        - skew(rA) * invIA * skew(rA)
        - skew(rB) * invIB * skew(rB);

    mat3 Kinv = K.inverse();

    vec3 lambda = -Kinv * (Cdot + bias);

    if (!parent->isStatic) {
        parent->curState.vel -= lambda * invMassA;
        parent->curState.angVel -= invIA * rA.cross(lambda);
    }

    if (!child->isStatic) {
        child->curState.vel += lambda * invMassB;
        child->curState.angVel += invIB * rB.cross(lambda);
    }

    quaternion qErr = child->curState.rot * parent->curState.rot.conjugate();
    vec3 angErr = 2.0 * vec3(qErr.x(), qErr.y(), qErr.z());

    vec3 angVelRel = child->curState.angVel - parent->curState.angVel;
    vec3 angBias = (beta / dt) * angErr;

    mat3 angK = invIA + invIB;
    mat3 angKinv = angK.inverse();

    vec3 angLambda = -angKinv * (angVelRel + angBias);

    if (!parent->isStatic)
        parent->curState.angVel -= invIA * angLambda;

    if (!child->isStatic)
        child->curState.angVel += invIB * angLambda;
}