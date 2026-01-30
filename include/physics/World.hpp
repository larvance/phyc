#pragma once
#include <chrono>
#include <deque>
#include <mutex>

#include "RigidBody.hpp"
#include "Spring.hpp"
#include "joints/Joint.hpp"

namespace PhyC {
    struct RigidBody;

    double approximateGravitationalAcceleration(double lat_rad, double alt_m);

    inline double getTime_s() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now.time_since_epoch()).count();
    }

    struct RayHit {
        RigidBody* body;
        vec3 point;
        double distance;
    };

    struct World {
    private:
        double lastUpdate = 0;
        double accumulatedTime = 0.0;

    public:
        mutable std::recursive_mutex worldMutex;
        const double timeStep = 0.001;
        double speed = 1.0;
        static constexpr int jointIterations = 10;

        std::deque<std::unique_ptr<RigidBody>> bodies;
        std::deque<std::unique_ptr<Joint>> joints;
        std::deque<std::unique_ptr<Spring>> springs;
        vec3 gravity{0.0, 0.0, -9.807};
        double pGravityLen = 9.807;

        FluidProperties sampleAirProperties(vec3 position) const;
        RigidBody& createRigidBody(const std::string& name = "RigidBody");
        RigidBody& addDebugVector(const vec3& pos, const vec3& vec);

        template <typename JointType, typename... Args>
        JointType& createJoint(Args&&... args) {
            std::lock_guard lock(worldMutex);
            static_assert(std::is_base_of_v<Joint, JointType>, "JointType must inherit from Joint");

            auto joint = std::make_unique<JointType>(std::forward<Args>(args)...);
            joints.push_back(std::move(joint));
            return *static_cast<JointType*>(joints.back().get());
        }

        Spring& createSpring(RigidBody& body, const vec3& targetPoint);
        Spring& createSpring(RigidBody& body, RigidBody& targetBody);
        bool hasRigidBody(const RigidBody& body) const;
        void removeRigidBody(RigidBody& body);
        bool raycast(RayHit& hit, double mouseX, double mouseY, double screenW, double screenH, const mat4& view,
                     const mat4& projection) const;
        void update();
        void step(double dt);
        void stepFixed();
        void reset();
        void resetTimer();
    };
}
