#pragma once
#include <chrono>
#include <deque>
#include <mutex>
#include <vector>

#include "RigidBody.hpp"

namespace PhyC {
    struct RigidBody;

    double approximateGravitationalAcceleration(double lat_rad, double alt_m);

    inline double getTime_s() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now.time_since_epoch()).count();
    }

    struct World {
    private:
        double lastUpdate = 0;
        double accumulatedTime = 0.0;
        mutable std::recursive_mutex worldMutex;

    public:
        const double timeStep = 0.001;
        double speed = 1.0;

        std::deque<std::unique_ptr<RigidBody>> bodies;
        vec3 gravity{0.0, 0.0, -9.807};

        AirProperties sampleAirProperties(vec3 position) const;

        RigidBody& createRigidBody(const std::string& name = "RigidBody") {
            std::lock_guard lock(worldMutex);
            auto body = std::make_unique<RigidBody>();
            body->name = name;
            body->world = this;
            bodies.push_back(std::move(body));
            return *bodies.back();
        }

        RigidBody& addDebugVector(const vec3& pos, const vec3& vec) {
            std::lock_guard lock(worldMutex);
            auto& body = createRigidBody("Vector");
            body.isStatic = true;
            body.curState.pos = pos;
            body.surface = {
                Surfaces::arrow(vec, {1, 0, 0}, 0.05, 0.1, 0.25)
            };
            return body;
        }

        void removeRigidBody(RigidBody& body) {
            std::lock_guard lock(worldMutex);
            std::erase_if(bodies, [&body](const std::unique_ptr<RigidBody>& b) {
                return b.get() == &body;
            });
        }

        void update() {
            std::lock_guard lock(worldMutex);
            double nowSeconds = getTime_s();
            if (lastUpdate == 0.0) {
                lastUpdate = nowSeconds;
                return;
            }
            double dt = (nowSeconds - lastUpdate) * speed;
            lastUpdate = nowSeconds;
            step(dt);
        }

        void step(double dt) {
            std::lock_guard lock(worldMutex);
            accumulatedTime += dt;
            while (accumulatedTime >= timeStep) {
                stepFixed();
                accumulatedTime -= timeStep;
            }
        }

        void stepFixed() const {
            std::lock_guard lock(worldMutex);
            for (auto& body : bodies) {
                body->beforeUpdate();
            }

            for (auto& body : bodies) {
                body->update(timeStep);
            }

            for (auto& body : bodies) {
                body->afterUpdate(timeStep);
            }
        }

        void reset() {
            std::lock_guard lock(worldMutex);
            bodies.clear();
            resetTimer();
        }

        void resetTimer() {
            std::lock_guard lock(worldMutex);
            lastUpdate = 0.0;
            accumulatedTime = 0.0;
        }
    };
}
