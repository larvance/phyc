#pragma once

#include "Joint.hpp"

namespace PhyC {
    struct SphericalJoint : Joint {
        bool enableConeLimit = false;
        double maxConeAngle = 0.0;

        bool enableTwistLimit = false;
        double minTwist = 0.0;
        double maxTwist = 0.0;

        SphericalJoint() {
            type = Type::Spherical;
        }

        void solve(double dt) override;
    };
}
