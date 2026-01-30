#pragma once

#include "Joint.hpp"

namespace PhyC {
    struct PrismaticJoint : Joint {
        vec3 axis = vec3(0, 0, 1);

        double position = 0.0;
        double velocity = 0.0;

        bool enableLimits = false;
        double minPosition = 0.0;
        double maxPosition = 0.0;

        bool enableMotor = false;
        double targetVelocity = 0.0;
        double maxForce = 0.0;

        double stiffness = 0.0;
        double damping = 0.0;

        PrismaticJoint() {
            type = Type::Prismatic;
        }

        void preStep(double dt) override;
        void solve(double dt) override;
    };
}
