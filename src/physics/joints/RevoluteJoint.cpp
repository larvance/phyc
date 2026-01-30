#include "physics/joints/RevoluteJoint.hpp"

void PhyC::RevoluteJoint::solve(double dt) {
    if (!parent || !child) return;
    if (parent->isStatic && child->isStatic) return;

    constexpr double beta = 0.2;

    vec3 axisWorld = parent->curState.rot * axis;

    vec3 rA = parent->curState.rot * parentAnchor;
    vec3 rB = child->curState.rot * childAnchor;

    vec3 pA = parent->curState.pos + rA;
    vec3 pB = child->curState.pos + rB;

    vec3 vA = parent->curState.vel + parent->curState.angVel.cross(rA);
    vec3 vB = child->curState.vel + child->curState.angVel.cross(rB);
    vec3 Cdot = vB - vA;
    vec3 Cpos = pB - pA;
    vec3 bias = (beta / dt) * Cpos;

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

    vec3 wRel = child->curState.angVel - parent->curState.angVel;
    vec3 wPerp = wRel - axisWorld * axisWorld.dot(wRel);
    vec3 angBias = (beta / dt) * wPerp;

    mat3 Kang = invIA + invIB;
    mat3 P = mat3::Identity() - axisWorld * axisWorld.transpose();
    mat3 KangProj = P * Kang * P;

    if (KangProj.determinant() != 0.0) {
        vec3 angLambda = -KangProj.inverse() * (wPerp + angBias);
        if (!parent->isStatic) parent->curState.angVel -= invIA * angLambda;
        if (!child->isStatic) child->curState.angVel += invIB * angLambda;
    }

    if (enableMotor) {
        double wAxis = axisWorld.dot(child->curState.angVel - parent->curState.angVel);
        double effMass = 1.0 / (axisWorld.dot(invIA * axisWorld) + axisWorld.dot(invIB * axisWorld));
        double impulse = std::clamp((targetVelocity - wAxis) * effMass, -maxTorque, maxTorque);

        if (!parent->isStatic) parent->curState.angVel -= invIA * (axisWorld * impulse);
        if (!child->isStatic) child->curState.angVel += invIB * (axisWorld * impulse);
    }

    if (enableLimits) {
        quaternion qRel = child->curState.rot * parent->curState.rot.conjugate();
        double currentAngle = 2.0 * atan2(
            axis.x() * qRel.x() + axis.y() * qRel.y() + axis.z() * qRel.z(),
            qRel.w()
        );

        double correction = 0.0;
        if (currentAngle < minAngle) correction = currentAngle - minAngle;
        if (currentAngle > maxAngle) correction = currentAngle - maxAngle;

        double biasLimit = (beta / dt) * correction;
        double effMass = 1.0 / (axisWorld.dot(invIA * axisWorld) + axisWorld.dot(invIB * axisWorld));
        double lambdaLimit = -biasLimit * effMass;

        if (!parent->isStatic) parent->curState.angVel -= invIA * (axisWorld * lambdaLimit);
        if (!child->isStatic) child->curState.angVel += invIB * (axisWorld * lambdaLimit);
    }
}