#pragma once
#include "physics/RigidBody.hpp"

namespace PhyC {
    struct RigidBody;

    struct JointLimit {
        double min;
        double max;
        double stiffness;
        double damping;

        double computeImpulse(double value, double velocity, double dt) const;
    };

    struct Joint {
        enum class Type {
            Fixed,
            Revolute,
            Prismatic,
            Spherical,
            Planar,
            Universal
        };

        Type type;

        RigidBody* parent = nullptr;
        RigidBody* child = nullptr;

        vec3 parentAnchor = vec3::Zero();
        vec3 childAnchor = vec3::Zero();

        quaternion parentFrame = quaternion::Identity();
        quaternion childFrame = quaternion::Identity();

        bool collideConnected = false;

        virtual ~Joint() = default;

        virtual void preStep(double dt) {
        }

        virtual void solve(double dt) = 0;
    };
}
