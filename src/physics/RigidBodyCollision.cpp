#include "RigidBody.hpp"
#include "World.hpp"

using namespace PhyC;

inline bool hasSeparation(const vec3& T, const vec3& axis, double rA, double rB) {
    double tProj = std::abs(T.dot(axis));
    return tProj > (rA + rB + 1e-8);
}

inline double projectBox(const vec3& extents, const mat3& basis, const vec3& axis) {
    return extents.x() * std::abs(basis.col(0).dot(axis)) +
        extents.y() * std::abs(basis.col(1).dot(axis)) +
        extents.z() * std::abs(basis.col(2).dot(axis));
}

vec3 getSupportPoint(const OBB& box, const vec3& pos, const mat3& rot, const vec3& dir) {
    mat3 totalRot = rot * box.localBasis;
    vec3 localDir = totalRot.transpose() * dir;

    constexpr double threshold = 1e-6;

    vec3 localSupport(
        (std::abs(localDir.x()) < threshold) ? 0.0 : (localDir.x() >= 0 ? box.halfExtents.x() : -box.halfExtents.x()),
        (std::abs(localDir.y()) < threshold) ? 0.0 : (localDir.y() >= 0 ? box.halfExtents.y() : -box.halfExtents.y()),
        (std::abs(localDir.z()) < threshold) ? 0.0 : (localDir.z() >= 0 ? box.halfExtents.z() : -box.halfExtents.z())
    );

    return pos + rot * box.centerOffset + totalRot * localSupport;
}

void RigidBody::precomputeColliderRadius() {
    pColliderRadius = 0.0;

    for (const auto& box : colliders) {
        vec3 corners[8] = {
            box.centerOffset + vec3{box.halfExtents.x(), box.halfExtents.y(), box.halfExtents.z()},
            box.centerOffset + vec3{box.halfExtents.x(), box.halfExtents.y(), -box.halfExtents.z()},
            box.centerOffset + vec3{box.halfExtents.x(), -box.halfExtents.y(), box.halfExtents.z()},
            box.centerOffset + vec3{box.halfExtents.x(), -box.halfExtents.y(), -box.halfExtents.z()},
            box.centerOffset + vec3{-box.halfExtents.x(), box.halfExtents.y(), box.halfExtents.z()},
            box.centerOffset + vec3{-box.halfExtents.x(), box.halfExtents.y(), -box.halfExtents.z()},
            box.centerOffset + vec3{-box.halfExtents.x(), -box.halfExtents.y(), box.halfExtents.z()},
            box.centerOffset + vec3{-box.halfExtents.x(), -box.halfExtents.y(), -box.halfExtents.z()}
        };

        for (const auto& corner : corners) {
            double dist = corner.norm();
            if (dist > pColliderRadius) {
                pColliderRadius = dist;
            }
        }
    }
}

bool testAxis(
    const vec3& axis, const vec3& T,
    const OBB& boxA, const mat3& RA,
    const OBB& boxB, const mat3& RB,
    double& minOverlap, vec3& bestAxis
) {
    if (axis.squaredNorm() < 1e-8) return false;

    vec3 n = axis.normalized();

    double rA = projectBox(boxA.halfExtents, RA, n);
    double rB = projectBox(boxB.halfExtents, RB, n);
    double dist = std::abs(T.dot(n));

    double overlap = (rA + rB) - dist;

    if (overlap <= 0) return true;

    if (overlap < minOverlap) {
        minOverlap = overlap;
        bestAxis = n;
        if (T.dot(bestAxis) < 0) {
            bestAxis = -bestAxis;
        }
    }
    return false;
}

enum class AxisOwner { None, BodyA, BodyB, EdgeEdge };

bool RigidBody::checkCollisionSAT(RigidBody& other, CollisionManifold& outManifold) const {
    double minOverlap = std::numeric_limits<double>::max();
    vec3 bestAxis = vec3::Zero();
    auto axisOwner = AxisOwner::None;

    auto tryAxis = [&](const vec3& axis, AxisOwner owner) -> bool {
        vec3 n = axis.normalized();
        if (n.squaredNorm() < 1e-8) return true;

        double rA = projectBox(colliders[0].halfExtents, curState.rot.toRotationMatrix() * colliders[0].localBasis, n);
        double rB = projectBox(other.colliders[0].halfExtents,
                               other.curState.rot.toRotationMatrix() * other.colliders[0].localBasis, n);

        vec3 T = (curState.pos + curState.rot * colliders[0].centerOffset) -
            (other.curState.pos + other.curState.rot * other.colliders[0].centerOffset);

        double dist = std::abs(T.dot(n));
        double overlap = (rA + rB) - dist;

        if (overlap <= 0) return false;

        constexpr double BIAS = 1e-4;

        bool isBetter = (overlap < minOverlap);

        if (std::abs(overlap - minOverlap) < BIAS &&
            axisOwner == AxisOwner::BodyA && owner == AxisOwner::BodyB) {
            isBetter = true;
        }

        if (isBetter) {
            minOverlap = overlap;
            bestAxis = n;
            axisOwner = owner;
            if (T.dot(bestAxis) < 0) {
                bestAxis = -bestAxis;
            }
        }
        return true;
    };

    mat3 RA = curState.rot.toRotationMatrix() * colliders[0].localBasis;
    for (int i = 0; i < 3; i++) {
        if (!tryAxis(RA.col(i), AxisOwner::BodyA)) return false;
    }

    mat3 RB = other.curState.rot.toRotationMatrix() * other.colliders[0].localBasis;
    for (int i = 0; i < 3; i++) {
        if (!tryAxis(RB.col(i), AxisOwner::BodyB)) return false;
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (!tryAxis(RA.col(i).cross(RB.col(j)), AxisOwner::EdgeEdge)) return false;
        }
    }

    outManifold.otherBody = &other;
    outManifold.depth = minOverlap;
    outManifold.normal = bestAxis;

    vec3 PA = curState.pos + (curState.rot * colliders[0].centerOffset);
    vec3 PB = other.curState.pos + (other.curState.rot * other.colliders[0].centerOffset);

    if (axisOwner == AxisOwner::BodyA) {
        outManifold.contactPoint = getSupportPoint(other.colliders[0], PB, RB, bestAxis);
        outManifold.contactPoint += bestAxis * (minOverlap * 0.5);
    } else if (axisOwner == AxisOwner::BodyB) {
        outManifold.contactPoint = getSupportPoint(colliders[0], PA, RA, -bestAxis);
        outManifold.contactPoint -= bestAxis * (minOverlap * 0.5);
    } else {
        vec3 pA = getSupportPoint(colliders[0], PA, RA, -bestAxis);
        vec3 pB = getSupportPoint(other.colliders[0], PB, RB, bestAxis);
        outManifold.contactPoint = (pA + pB) * 0.5;
    }

    return true;
}

void RigidBody::alignCollidingBodies() {
    if (!world) return;

    collisionCache.clear();

    for (auto& otherPtr : world->bodies) {
        RigidBody& other = *otherPtr;
        if (id == other.id) continue;

        double r = pColliderRadius + other.pColliderRadius;
        if ((curState.pos - other.curState.pos).squaredNorm() > r * r) continue;

        CollisionManifold manifold;
        if (checkCollisionSAT(other, manifold)) {
            collisionCache.push_back(manifold);

            double moveRatio = other.isStatic ? 1.0 : 0.5;

            constexpr double slop = 0.005; // 5mm
            double correction = std::max(0.0, manifold.depth - slop);

            constexpr double percent = 0.4;

            vec3 correctionVec = manifold.normal * (correction * percent * moveRatio);

            if (correction > 0.0) {
                curState.pos += correctionVec;
            }
        }
    }
}

void RigidBody::computeCollisionReactions(double dt) {
    for (const auto& manifold : collisionCache) resolveCollision(manifold, dt);
}

void RigidBody::resolveCollision(const CollisionManifold& manifold, double dt) {
    RigidBody& A = *this;
    RigidBody& B = *manifold.otherBody;

    vec3 n = manifold.normal;
    vec3 rA = manifold.contactPoint - A.curState.pos;
    vec3 rB = manifold.contactPoint - B.curState.pos;

    // V_point = V_cm + (omega x r)
    vec3 vA = A.curState.vel + A.curState.angVel.cross(rA);
    vec3 vB = B.curState.vel + B.curState.angVel.cross(rB);
    vec3 relVel = vA - vB;

    double velAlongNormal = relVel.dot(n);

    if (velAlongNormal > 0) return;

    const auto& matA = A.colliders[0].material;
    const auto& matB = B.colliders[0].material;

    double e = (matA.restitution + matB.restitution) * 0.5;
    if (std::abs(velAlongNormal) < (9.81 * dt * 2.0)) {
        e = 0.0;
    }

    double friction = (matA.friction + matB.friction) * 0.5;

    double restingThreshold = -2.0 * 9.81 * dt;

    if (velAlongNormal > restingThreshold) e = 0.0;

    // j = -(1 + e) * v_rel_norm / (1/mA + 1/mB + ... rotational terms ...)
    double invMassSum = A.pInvMass + B.pInvMass;
    // Rotational Inertia Terms: (r x n)^T * I^-1 * (r x n)
    vec3 rAxn = rA.cross(n);
    vec3 rBxn = rB.cross(n);
    double termA = rAxn.dot(A.invWorldInertia * rAxn);
    double termB = rBxn.dot(B.invWorldInertia * rBxn);
    double j = -(1.0 + e) * velAlongNormal;
    j /= (invMassSum + termA + termB);

    vec3 impulse = j * n;

    A.curState.vel += impulse * A.pInvMass;
    A.curState.angVel += A.invWorldInertia * rA.cross(impulse);

    if (!B.isStatic) {
        B.curState.vel -= impulse * B.pInvMass;
        B.curState.angVel -= B.invWorldInertia * rB.cross(impulse);
    }

    vec3 t = relVel - (velAlongNormal * n);
    double tangentLen = t.norm();
    if (tangentLen > 1e-3) {
        t /= tangentLen;

        vec3 rAxt = rA.cross(t);
        vec3 rBxt = rB.cross(t);
        double termAt = rAxt.dot(A.invWorldInertia * rAxt);
        double termBt = rBxt.dot(B.invWorldInertia * rBxt);

        double jt = -relVel.dot(t);
        jt /= (invMassSum + termAt + termBt);

        // Coulomb Friction Clamp
        if (std::abs(jt) > j * friction) {
            jt = std::copysign(j * friction, jt);
        }

        vec3 frictionImpulse = jt * t;

        A.curState.vel += frictionImpulse * A.pInvMass;
        A.curState.angVel += A.invWorldInertia * rA.cross(frictionImpulse);

        if (!B.isStatic) {
            B.curState.vel -= frictionImpulse * B.pInvMass;
            B.curState.angVel -= B.invWorldInertia * rB.cross(frictionImpulse);
        }
    }
}
