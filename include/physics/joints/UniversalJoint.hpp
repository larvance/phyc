#pragma once

#include "Joint.hpp"

namespace PhyC {
    struct UniversalJoint : Joint {
        vec3 axis1 = vec3(1, 0, 0);
        vec3 axis2 = vec3(0, 1, 0);

        double angle1 = 0.0;
        double angle2 = 0.0;

        UniversalJoint() {
            type = Type::Universal;
        }

        void solve(double dt) override;
    };
}
