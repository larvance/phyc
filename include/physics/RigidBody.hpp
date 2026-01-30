#pragma once
#include <atomic>
#include <string>
#include <vector>

#include "physics/FluidProperties.hpp"
#include "physics/OrientedBoundingBox.hpp"
#include "RigidBodyMass.hpp"
#include "SurfaceTriangleElement.hpp"
#include "phyc.hpp"
#include "joints/Joint.hpp"

namespace PhyC {
    struct World;

    extern std::atomic<unsigned long long> next_id;

    inline auto getNextId() {
        return next_id.fetch_add(1, std::memory_order_relaxed) + 1;
    }

    struct State {
        vec3 pos = vec3::Zero();
        vec3 vel = vec3::Zero();
        quaternion rot = quaternion::Identity();
        vec3 angVel = vec3::Zero();
    };

    struct FTState : State {
    private:
        vec3 _force = vec3::Zero();
        vec3 _torque = vec3::Zero();

    public:
        FTState() {
        }

        explicit FTState(const State& base) :
            State(base),
            _force(vec3::Zero()),
            _torque(vec3::Zero()) {
        }

        constexpr const vec3& force() const {
            return _force;
        }

        constexpr const vec3& torque() const {
            return _torque;
        }

        void applyForce(const vec3& f) {
            _force += f;
        }

        void applyForce(const vec3& f, const vec3& applicationPoint) {
            _force += f;
            _torque += (applicationPoint - pos).cross(f);
        }

        void applyForceDist(const vec3& f, const vec3& r) {
            _force += f;
            _torque += r.cross(f);
        }

        void applyTorque(const vec3& t) {
            _torque += t;
        }
    };

    struct Derivative {
        vec3 dPos = vec3::Zero();
        vec3 dVel = vec3::Zero();
        quaternion dRot{0, 0, 0, 0};
        vec3 dAngVel = vec3::Zero();
    };

    struct RigidBody {
        unsigned long long id = getNextId();
        std::string name = "RigidBody";
        bool isStatic = false;

        RigidBody() {
        }

        RigidBody(const RigidBody&) = delete;
        RigidBody& operator=(const RigidBody&) = delete;

        State curState{};

        World* world = nullptr;

        Surface surface{};                        // Defines mass and aerodynamic surfaces, are rendered
        std::vector<RigidBodyMass> extraMasses{}; // Point masses added for mass/inertia tuning, not rendered
        std::vector<OBB> colliders{};             // Bounding boxes only for collision detection, not rendered

        // Aerodynamic coefficients
        double Clp = -0.4;
        double Cmq = -10.0;
        double Cnr = -0.2;

        /////////////////////////////////////////////////////

        // Values computed once and cached:
        bool precomputed = false;
        double pColliderRadius = 0.0;
        vec3 pStableCenterOfMass = vec3::Zero();
        double pMass = 0.0;
        double pInvMass = 0.0;
        mat3 pBodyInertia = mat3::Zero();
        mat3 pInvBodyInertia = mat3::Zero();
        void precomputeMass();
        void precomputeInertia();
        void precomputeColliderRadius();

        void precompute() {
            if (precomputed) return;
            precomputed = true;
            precomputeMass();
            precomputeInertia();
            precomputeColliderRadius();
        }

        /////////////////////////////////////////////////////

        // Computed before every update:
        mat3 worldInertia = mat3::Zero();
        mat3 invWorldInertia = mat3::Zero();
        void computeWorldInertia();

        void beforeUpdate() {
            precompute();
            computeWorldInertia();
        }

        /////////////////////////////////////////////////////

        struct CollisionManifold {
            RigidBody* otherBody;
            vec3 normal;
            vec3 contactPoint;
            double depth;
        };

        std::vector<CollisionManifold> collisionCache{};

        void evaluateRK4(Derivative& out, const State& initial, const Derivative& d, double dt) const;
        void integrateRK4(double dt);
        void integrateEuler(double dt);
        void computeForcesAt(FTState& state) const;
        void computeGravityForce(FTState& state) const;
        void computeAeroSurfaceForces(FTState& state, const FluidProperties& air) const;
        void computeAeroDampingTorque(FTState& state, const FluidProperties& air) const;
        void alignCollidingBodies();
        void resolveCollision(const CollisionManifold& manifold, double dt);
        bool checkCollisionSAT(RigidBody& other, CollisionManifold& outManifold) const;
        void update(double dt);

        /////////////////////////////////////////////////////

        void computeCollisionReactions(double dt);

        void afterUpdate(double dt) {
            computeCollisionReactions(dt);
        }

        /////////////////////////////////////////////////////

        static double solveLocalPressure(
            const vec3& normal,
            const vec3& relativeVelocity,
            const FluidProperties& air
        );

        static vec3 solveLocalShearStress(
            const vec3& normal,
            const vec3& relativeVelocity,
            const FluidProperties& air
        );

        template <typename JointType>
        JointType& createJoint(RigidBody& target);

        void remove();
    };
}
