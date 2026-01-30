#pragma once

#include "Joint.hpp"

namespace PhyC {
    struct RevoluteJoint : Joint {
        vec3 axis = vec3(0, 0, 1);

        double angle = 0.0;
        double angularVelocity = 0.0;

        bool enableLimits = false;
        double minAngle = 0.0;
        double maxAngle = 0.0;

        bool enableMotor = false;
        double targetVelocity = 0.0;
        double maxTorque = 0.0;

        double stiffness = 0.0;
        double damping = 0.0;

        RevoluteJoint() {
            type = Type::Revolute;
        }

        void solve(double dt) override;
    };
}
