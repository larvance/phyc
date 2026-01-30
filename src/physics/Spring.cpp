#include "physics/Spring.hpp"

void PhyC::Spring::applyForce(double dt) const {
    if (!bodyA) return;

    vec3 rA = bodyA->curState.rot * anchorA;
    vec3 pA = bodyA->curState.pos + rA;

    vec3 pB, velB;

    if (bodyB) {
        vec3 rB = bodyB->curState.rot * anchorB;
        pB = bodyB->curState.pos + rB;
        velB = bodyB->curState.vel + bodyB->curState.angVel.cross(rB);
    } else {
        pB = worldTargetPos;
        velB = vec3(0, 0, 0);
    }

    vec3 delta = pB - pA;
    double dist = delta.norm();

    if (dist < 1e-6) return;

    vec3 dir = delta / dist;

    double springForce = stiffness * (dist - restLength);
    if (springForce < 0.0) springForce = 0.0;

    vec3 velA = bodyA->curState.vel + bodyA->curState.angVel.cross(rA);
    double relVel = (velB - velA).dot(dir);
    double dampingForce = damping * relVel;

    double totalForce = springForce + dampingForce;
    vec3 forceVec = dir * totalForce;

    if (!bodyA->isStatic) {
        bodyA->curState.vel += (forceVec * dt) * bodyA->pInvMass;
        bodyA->curState.angVel += (bodyA->invWorldInertia * rA.cross(forceVec)) * dt;
    }

    if (bodyB && !bodyB->isStatic) {
        bodyB->curState.vel -= (forceVec * dt) * bodyB->pInvMass;
        bodyB->curState.angVel -= (bodyB->invWorldInertia * (bodyB->curState.rot * anchorB).cross(forceVec)) * dt;
    }
}
