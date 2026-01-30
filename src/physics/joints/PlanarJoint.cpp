#include "physics/joints/PlanarJoint.hpp"

void PhyC::PlanarJoint::solve(double dt) {
    if (!parent || !child) return;
    if (parent->isStatic && child->isStatic) return;

    constexpr double beta = 0.2;

    vec3 n = normal.normalized();

    vec3 rA = parent->curState.rot * parentAnchor;
    vec3 rB = child->curState.rot * childAnchor;

    vec3 pA = parent->curState.pos + rA;
    vec3 pB = child->curState.pos + rB;

    vec3 vA = parent->curState.vel + parent->curState.angVel.cross(rA);
    vec3 vB = child->curState.vel + child->curState.angVel.cross(rB);

    double C = n.dot(pB - pA);
    double Cdot = n.dot(vB - vA);

    double invMassA = parent->isStatic ? 0.0 : parent->pInvMass;
    double invMassB = child->isStatic ? 0.0 : child->pInvMass;

    mat3 invIA = parent->isStatic ? mat3::Zero() : parent->invWorldInertia;
    mat3 invIB = child->isStatic ? mat3::Zero() : child->invWorldInertia;

    double bias = (beta / dt) * C;

    vec3 rnA = rA.cross(n);
    vec3 rnB = rB.cross(n);

    double k =
        invMassA + invMassB +
        rnA.dot(invIA * rnA) +
        rnB.dot(invIB * rnB);

    if (k > 0.0) {
        double lambda = -(Cdot + bias) / k;
        vec3 impulse = lambda * n;

        if (!parent->isStatic) {
            parent->curState.vel -= impulse * invMassA;
            parent->curState.angVel -= invIA * rA.cross(impulse);
        }

        if (!child->isStatic) {
            child->curState.vel += impulse * invMassB;
            child->curState.angVel += invIB * rB.cross(impulse);
        }
    }

    vec3 wRel = child->curState.angVel - parent->curState.angVel;
    vec3 wPerp = wRel - n * n.dot(wRel);

    vec3 angBias = (beta / dt) * wPerp;

    mat3 angK = invIA + invIB;

    mat3 P = mat3::Identity() - outerProduct(n, n);
    mat3 Kproj = P * angK * P;

    if (Kproj.determinant() != 0.0) {
        vec3 angLambda = -Kproj.inverse() * (wPerp + angBias);

        if (!parent->isStatic)
            parent->curState.angVel -= invIA * angLambda;

        if (!child->isStatic)
            child->curState.angVel += invIB * angLambda;
    }
}