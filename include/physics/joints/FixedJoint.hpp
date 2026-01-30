#pragma once

#include "Joint.hpp"

namespace PhyC {
    struct FixedJoint : Joint {
        vec3 initialOffset;
        quaternion initialRotation;

        FixedJoint() {
            type = Type::Fixed;
        }

        void solve(double dt) override;
    };
}
