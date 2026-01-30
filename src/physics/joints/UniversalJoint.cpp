#include "physics/joints/UniversalJoint.hpp"

void PhyC::UniversalJoint::solve(double dt) {
    if (!parent || !child) return;
    if (parent->isStatic && child->isStatic) return;

    constexpr double beta = 0.2;

    vec3 rA = parent->curState.rot * parentAnchor;
    vec3 rB = child->curState.rot * childAnchor;

    vec3 pA = parent->curState.pos + rA;
    vec3 pB = child->curState.pos + rB;

    vec3 delta = pB - pA;
    vec3 vA = parent->curState.vel + parent->curState.angVel.cross(rA);
    vec3 vB = child->curState.vel + child->curState.angVel.cross(rB);
    vec3 Cdot = vB - vA;

    vec3 bias = (beta / dt) * delta;

    double invMassA = parent->isStatic ? 0.0 : parent->pInvMass;
    double invMassB = child->isStatic ? 0.0 : child->pInvMass;
    mat3 invIA = parent->isStatic ? mat3::Zero() : parent->invWorldInertia;
    mat3 invIB = child->isStatic ? mat3::Zero() : child->invWorldInertia;

    mat3 K = mat3::Identity() * (invMassA + invMassB)
        - skew(rA) * invIA * skew(rA)
        - skew(rB) * invIB * skew(rB);

    vec3 lambda = -K.inverse() * (Cdot + bias);

    if (!parent->isStatic) {
        parent->curState.vel -= lambda * invMassA;
        parent->curState.angVel -= invIA * rA.cross(lambda);
    }
    if (!child->isStatic) {
        child->curState.vel += lambda * invMassB;
        child->curState.angVel += invIB * rB.cross(lambda);
    }

    vec3 axisWorld1 = parent->curState.rot * axis1;
    vec3 axisWorld2 = parent->curState.rot * axis2;

    vec3 wRel = child->curState.angVel - parent->curState.angVel;

    vec3 lockedAxis = axisWorld1.cross(axisWorld2);
    vec3 wLocked = lockedAxis * lockedAxis.dot(wRel);

    vec3 angBias = (beta / dt) * wLocked;

    mat3 Kang = invIA + invIB;
    mat3 P = outerProduct(lockedAxis, lockedAxis);
    mat3 KangProj = P * Kang * P;

    if (KangProj.determinant() != 0.0) {
        vec3 angLambda = -KangProj.inverse() * (wLocked + angBias);
        if (!parent->isStatic) parent->curState.angVel -= invIA * angLambda;
        if (!child->isStatic) child->curState.angVel += invIB * angLambda;
    }

    angle1 = axisWorld1.dot(child->curState.angVel - parent->curState.angVel);
    angle2 = axisWorld2.dot(child->curState.angVel - parent->curState.angVel);
}
