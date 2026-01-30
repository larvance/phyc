#include "physics/joints/SphericalJoint.hpp"

#include <cmath>

void PhyC::SphericalJoint::solve(double dt) {
    if (!parent || !child) return;
    if (parent->isStatic && child->isStatic) return;

    constexpr double beta = 0.2;
    constexpr double epsilon = 1e-6;

    double invMassA = parent->isStatic ? 0.0 : parent->pInvMass;
    double invMassB = child->isStatic ? 0.0 : child->pInvMass;

    mat3 invIA = parent->isStatic ? mat3::Zero() : parent->invWorldInertia;
    mat3 invIB = child->isStatic ? mat3::Zero() : child->invWorldInertia;
    mat3 sumInvI = invIA + invIB;

    quaternion qGlobalA = parent->curState.rot * parentFrame;
    quaternion qGlobalB = child->curState.rot * childFrame;

    {
        vec3 rA = parent->curState.rot * parentAnchor;
        vec3 rB = child->curState.rot * childAnchor;

        vec3 pA = parent->curState.pos + rA;
        vec3 pB = child->curState.pos + rB;

        vec3 C = pB - pA;

        vec3 vA = parent->curState.vel + parent->curState.angVel.cross(rA);
        vec3 vB = child->curState.vel + child->curState.angVel.cross(rB);
        vec3 Cdot = vB - vA;

        vec3 bias = (beta / dt) * C;

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
    }

    if (enableConeLimit || enableTwistLimit) {
        quaternion qRel = qGlobalA.conjugate() * qGlobalB;

        quaternion qTwist(qRel.w(), qRel.x(), 0.0, 0.0);

        double twistMagSq = qTwist.w() * qTwist.w() + qTwist.x() * qTwist.x();
        if (twistMagSq > epsilon * epsilon) {
            double invMag = 1.0 / std::sqrt(twistMagSq);
            qTwist = quaternion(qTwist.w() * invMag, qTwist.x() * invMag, 0.0, 0.0);
        } else {
            qTwist = quaternion::Identity();
        }

        quaternion qSwing = qRel * qTwist.conjugate();

        if (enableTwistLimit) {
            double twistAngle = 2.0 * std::atan2(qTwist.x(), qTwist.w());

            if (twistAngle > M_PI) twistAngle -= 2.0 * M_PI;
            if (twistAngle < -M_PI) twistAngle += 2.0 * M_PI;

            double C_twist = 0.0;

            if (twistAngle < minTwist) {
                C_twist = twistAngle - minTwist;
            } else if (twistAngle > maxTwist) {
                C_twist = twistAngle - maxTwist;
            }

            if (std::abs(C_twist) > 0.0) {
                vec3 axis = qGlobalA * vec3(1, 0, 0);

                vec3 angVelRel = child->curState.angVel - parent->curState.angVel;
                double Cdot_twist = axis.dot(angVelRel);

                double bias_twist = (beta / dt) * C_twist;

                double kScalar = axis.dot(sumInvI * axis);

                if (kScalar > epsilon) {
                    double lambdaTwist = -(Cdot_twist + bias_twist) / kScalar;
                    vec3 impulseTwist = axis * lambdaTwist;

                    if (!parent->isStatic) parent->curState.angVel -= invIA * impulseTwist;
                    if (!child->isStatic) child->curState.angVel += invIB * impulseTwist;
                }
            }
        }

        if (enableConeLimit) {
            double swingAngle = 2.0 * std::acos(std::clamp(qSwing.w(), -1.0, 1.0));

            if (swingAngle > maxConeAngle) {
                double C_cone = swingAngle - maxConeAngle;

                vec3 localSwingAxis(qSwing.x(), qSwing.y(), qSwing.z());
                double len = localSwingAxis.norm();

                if (len > epsilon) {
                    localSwingAxis = localSwingAxis * (1.0 / len);

                    vec3 axis = qGlobalA * localSwingAxis;

                    vec3 angVelRel = child->curState.angVel - parent->curState.angVel;
                    double Cdot_cone = axis.dot(angVelRel);

                    double bias_cone = (beta / dt) * C_cone;

                    double kScalar = axis.dot(sumInvI * axis);

                    if (kScalar > epsilon) {
                        double lambdaCone = -(Cdot_cone + bias_cone) / kScalar;
                        vec3 impulseCone = axis * lambdaCone;

                        if (!parent->isStatic) parent->curState.angVel -= invIA * impulseCone;
                        if (!child->isStatic) child->curState.angVel += invIB * impulseCone;
                    }
                }
            }
        }
    }
}
