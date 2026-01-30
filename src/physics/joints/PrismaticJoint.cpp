#include "physics/joints/PrismaticJoint.hpp"
#include <cmath>
#include <algorithm>

using namespace PhyC;

static void computeBasis(const vec3& n, vec3& u, vec3& v) {
    if (std::abs(n.x()) >= 1.0 / sqrt(3))
        u = vec3(n.y(), -n.x(), 0.0);
    else
        u = vec3(0.0, n.z(), -n.y());
    u.normalize();
    v = n.cross(u);
}

void PrismaticJoint::preStep(double dt) {
    if (!parent || !child) return;

    vec3 n = parent->curState.rot * axis;
    n.normalize();

    vec3 rA = parent->curState.rot * parentAnchor;
    vec3 rB = child->curState.rot * childAnchor;
    vec3 pA = parent->curState.pos + rA;
    vec3 pB = child->curState.pos + rB;

    vec3 d = pB - pA;
    vec3 vRel = (child->curState.vel + child->curState.angVel.cross(rB))
        - (parent->curState.vel + parent->curState.angVel.cross(rA));

    this->position = d.dot(n);
    this->velocity = vRel.dot(n);
}

void PrismaticJoint::solve(double dt) {
    if (!parent || !child) return;
    if (parent->isStatic && child->isStatic) return;

    constexpr double beta = 0.2;
    constexpr double epsilon = 1e-6;

    double invMassA = parent->isStatic ? 0.0 : parent->pInvMass;
    double invMassB = child->isStatic ? 0.0 : child->pInvMass;
    mat3 invIA = parent->isStatic ? mat3::Zero() : parent->invWorldInertia;
    mat3 invIB = child->isStatic ? mat3::Zero() : child->invWorldInertia;

    vec3 rA = parent->curState.rot * parentAnchor;
    vec3 rB = child->curState.rot * childAnchor;

    vec3 n = parent->curState.rot * axis;
    n.normalize();

    vec3 u, v;
    computeBasis(n, u, v);

    {
        quaternion qErr = child->curState.rot * parent->curState.rot.conjugate();
        vec3 angErr = 2.0 * vec3(qErr.x(), qErr.y(), qErr.z());

        vec3 angVelRel = child->curState.angVel - parent->curState.angVel;
        vec3 angBias = (beta / dt) * angErr;

        mat3 angK = invIA + invIB;

        vec3 angLambda = -angK.inverse() * (angVelRel + angBias);

        if (!parent->isStatic) parent->curState.angVel -= invIA * angLambda;
        if (!child->isStatic) child->curState.angVel += invIB * angLambda;
    }

    {
        vec3 pA = parent->curState.pos + rA;
        vec3 pB = child->curState.pos + rB;
        vec3 delta = pB - pA;

        vec3 vA = parent->curState.vel + parent->curState.angVel.cross(rA);
        vec3 vB = child->curState.vel + child->curState.angVel.cross(rB);
        vec3 vRel = vB - vA;

        vec2 C_ortho(delta.dot(u), delta.dot(v));
        vec2 Cdot_ortho(vRel.dot(u), vRel.dot(v));
        vec2 bias_ortho = (beta / dt) * C_ortho;

        mat3 K_full = mat3::Identity() * (invMassA + invMassB)
            - skew(rA) * invIA * skew(rA)
            - skew(rB) * invIB * skew(rB);

        mat2 K2;
        K2(0, 0) = u.dot(K_full * u);
        K2(0, 1) = u.dot(K_full * v);
        K2(1, 0) = v.dot(K_full * u);
        K2(1, 1) = v.dot(K_full * v);

        vec2 lambda2 = -K2.inverse() * (Cdot_ortho + bias_ortho);
        vec3 impulse = u * lambda2[0] + v * lambda2[1];

        if (!parent->isStatic) {
            parent->curState.vel -= impulse * invMassA;
            parent->curState.angVel -= invIA * rA.cross(impulse);
        }
        if (!child->isStatic) {
            child->curState.vel += impulse * invMassB;
            child->curState.angVel += invIB * rB.cross(impulse);
        }
    }

    {
        vec3 vA = parent->curState.vel + parent->curState.angVel.cross(rA);
        vec3 vB = child->curState.vel + child->curState.angVel.cross(rB);
        vec3 vRel = vB - vA;

        double currentPos = this->position;
        double currentVel = vRel.dot(n);

        double C = 0.0;
        double bias = 0.0;
        double effectiveMass = n.dot((mat3::Identity() * (invMassA + invMassB)
            - skew(rA) * invIA * skew(rA)
            - skew(rB) * invIB * skew(rB)) * n);

        if (effectiveMass < epsilon) return;

        double lambda = 0.0;
        bool applyImpulse = false;

        if (enableLimits) {
            if (currentPos < minPosition) {
                C = currentPos - minPosition;
                bias = (beta / dt) * C;
                applyImpulse = true;
            } else if (currentPos > maxPosition) {
                C = currentPos - maxPosition;
                bias = (beta / dt) * C;
                applyImpulse = true;
            }
        }

        if (enableMotor && !applyImpulse) {
            double Cdot = currentVel - targetVelocity;
            lambda = -Cdot / effectiveMass;

            double maxImpulse = maxForce * dt;
            lambda = std::clamp(lambda, -maxImpulse, maxImpulse);

            applyImpulse = true;
        } else if (applyImpulse) {
            double Cdot = currentVel;
            lambda = -(Cdot + bias) / effectiveMass;

            if (currentPos < minPosition) lambda = std::max(0.0, lambda);
            if (currentPos > maxPosition) lambda = std::min(0.0, lambda);
        }

        if (stiffness > 0.0 || damping > 0.0) {
            double springForce = -stiffness * currentPos - damping * currentVel;
            double springImpulse = springForce * dt;

            lambda += springImpulse;
            applyImpulse = true;
        }

        if (applyImpulse) {
            vec3 impulse = n * lambda;
            if (!parent->isStatic) {
                parent->curState.vel -= impulse * invMassA;
                parent->curState.angVel -= invIA * rA.cross(impulse);
            }
            if (!child->isStatic) {
                child->curState.vel += impulse * invMassB;
                child->curState.angVel += invIB * rB.cross(impulse);
            }
        }
    }
}
