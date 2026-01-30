#include "physics/RigidBody.hpp"
#include "physics/World.hpp"

using namespace PhyC;

inline vec3 absVec(const vec3& v) {
    return vec3(std::abs(v.x()), std::abs(v.y()), std::abs(v.z()));
}

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
    vec3 localDir = rot.transpose() * dir;

    vec3 localSupport(
        std::copysign(box.halfExtents.x(), localDir.x()),
        std::copysign(box.halfExtents.y(), localDir.y()),
        std::copysign(box.halfExtents.z(), localDir.z())
    );

    return pos + rot * (box.centerOffset + localSupport);
}

void RigidBody::precomputeColliderRadius() {
    pColliderRadius = 0.0;

    for (const auto& box : colliders) {
        vec3 maxCorner = absVec(box.centerOffset) + box.halfExtents;
        double dist = maxCorner.norm();
        if (dist > pColliderRadius) {
            pColliderRadius = dist;
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

    const mat3 RA = curState.rot.toRotationMatrix();
    const mat3 RB = other.curState.rot.toRotationMatrix();

    const mat3 basisA = RA * colliders[0].localBasis;
    const mat3 basisB = RB * other.colliders[0].localBasis;

    vec3 PA = curState.pos + RA * colliders[0].centerOffset;
    vec3 PB = other.curState.pos + RB * other.colliders[0].centerOffset;
    vec3 T = PA - PB;

    auto tryAxis = [&](const vec3& axis, AxisOwner owner) -> bool {
        double l2 = axis.squaredNorm();
        if (l2 < 1e-8) return true;

        double invLen = 1.0 / std::sqrt(l2);
        vec3 n = axis * invLen;

        double rA = projectBox(colliders[0].halfExtents, basisA, n);
        double rB = projectBox(other.colliders[0].halfExtents, basisB, n);
        double dist = std::abs(T.dot(n));

        double overlap = (rA + rB) - dist;

        if (overlap <= 0) return false;

        bool isBetter = (overlap < minOverlap);

        constexpr double BIAS = 1e-4;
        if (std::abs(overlap - minOverlap) < BIAS &&
            axisOwner == AxisOwner::EdgeEdge && owner != AxisOwner::EdgeEdge) {
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

    for (int i = 0; i < 3; i++) {
        if (!tryAxis(basisA.col(i), AxisOwner::BodyA)) return false;
    }

    for (int i = 0; i < 3; i++) {
        if (!tryAxis(basisB.col(i), AxisOwner::BodyB)) return false;
    }

    for (int i = 0; i < 3; i++) {
        vec3 u = basisA.col(i);
        for (int j = 0; j < 3; j++) {
            vec3 v = basisB.col(j);
            if (!tryAxis(u.cross(v), AxisOwner::EdgeEdge)) return false;
        }
    }

    outManifold.otherBody = &other;
    outManifold.depth = minOverlap;
    outManifold.normal = bestAxis;

    if (axisOwner == AxisOwner::BodyA) {
        outManifold.contactPoint = getSupportPoint(other.colliders[0], PB, RB, -bestAxis);
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
    collisionCache.reserve(16);

    for (auto& otherPtr : world->bodies) {
        RigidBody& other = *otherPtr;
        if (id == other.id) continue;

        double r = pColliderRadius + other.pColliderRadius;
        vec3 delta = curState.pos - other.curState.pos;
        if (delta.squaredNorm() > r * r) continue;

        CollisionManifold manifold;
        if (checkCollisionSAT(other, manifold)) {
            collisionCache.push_back(manifold);

            constexpr double slop = 0.005; // 5mm tolerance
            double correction = std::max(0.0, manifold.depth - slop);

            if (correction > 0.0) {
                constexpr double correctionPercent = 0.4;

                double imA = pInvMass;
                double imB = other.isStatic ? 0.0 : other.pInvMass;
                double totalInvMass = imA + imB;

                if (totalInvMass > 1e-8) {
                    vec3 separation = manifold.normal * (correction * correctionPercent);

                    curState.pos += separation * (imA / totalInvMass);

                    if (!other.isStatic) {
                        other.curState.pos -= separation * (imB / totalInvMass);
                    }
                }
            }
        }
    }
}

void RigidBody::computeCollisionReactions(double dt) {
    for (const auto& manifold : collisionCache) {
        resolveCollision(manifold, dt);
    }
}

void RigidBody::resolveCollision(const CollisionManifold& manifold, double dt) {
    RigidBody& A = *this;
    RigidBody& B = *manifold.otherBody;

    const auto& matA = A.colliders[0].material;
    const auto& matB = B.colliders[0].material;

    vec3 n = manifold.normal;
    vec3 rA = manifold.contactPoint - A.curState.pos;
    vec3 rB = manifold.contactPoint - B.curState.pos;

    // V_point = V_cm + (omega x r)
    vec3 vA = A.curState.vel + A.curState.angVel.cross(rA);
    vec3 vB = B.curState.vel + B.curState.angVel.cross(rB);
    vec3 relVel = vA - vB;

    double velAlongNormal = relVel.dot(n);

    if (velAlongNormal > 0) return;

    double e = (matA.restitution + matB.restitution) * 0.5;
    if (world) {
        double restingThreshold = 0.5; // 2.0 * world->pGravityLen * dt;
        if (velAlongNormal > -restingThreshold) {
            e = 0.0;
        }
    }

    double invMassSum = A.pInvMass + B.pInvMass;

    // Rotational Inertia Terms: (r x n)^T * I^-1 * (r x n)
    vec3 rAxn = rA.cross(n);
    vec3 rBxn = rB.cross(n);
    vec3 iA_rAxn = A.invWorldInertia * rAxn;
    vec3 iB_rBxn = B.invWorldInertia * rBxn;

    double termA = rAxn.dot(iA_rAxn);
    double termB = rBxn.dot(iB_rBxn);

    double j = -(1.0 + e) * velAlongNormal;
    j /= (invMassSum + termA + termB);

    vec3 impulse = j * n;

    // Apply Normal Impulse
    A.curState.vel += impulse * A.pInvMass;
    A.curState.angVel += A.invWorldInertia * rA.cross(impulse);

    if (!B.isStatic) {
        B.curState.vel -= impulse * B.pInvMass;
        B.curState.angVel -= B.invWorldInertia * rB.cross(impulse);
    }

    vA = A.curState.vel + A.curState.angVel.cross(rA);
    vB = B.curState.vel + B.curState.angVel.cross(rB);
    relVel = vA - vB;

    vec3 t = relVel - (n * relVel.dot(n));
    double tangentLenSq = t.squaredNorm();

    if (tangentLenSq > 1e-6) {
        t /= std::sqrt(tangentLenSq);

        vec3 rAxt = rA.cross(t);
        vec3 rBxt = rB.cross(t);
        double termAt = rAxt.dot(A.invWorldInertia * rAxt);
        double termBt = rBxt.dot(B.invWorldInertia * rBxt);

        double jt = -relVel.dot(t);
        jt /= (invMassSum + termAt + termBt);

        // Coulomb Friction
        double friction = (matA.friction + matB.friction) * 0.5;
        double maxJt = j * friction;

        // Clamp friction impulse
        if (jt > maxJt) jt = maxJt;
        else if (jt < -maxJt) jt = -maxJt;

        vec3 frictionImpulse = jt * t;

        A.curState.vel += frictionImpulse * A.pInvMass;
        A.curState.angVel += A.invWorldInertia * rA.cross(frictionImpulse);

        if (!B.isStatic) {
            B.curState.vel -= frictionImpulse * B.pInvMass;
            B.curState.angVel -= B.invWorldInertia * rB.cross(frictionImpulse);
        }
    }
}
