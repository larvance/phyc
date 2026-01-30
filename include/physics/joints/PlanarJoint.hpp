#pragma once

#include "Joint.hpp"

namespace PhyC {
    struct PlanarJoint : Joint {
        vec3 normal = vec3(0, 1, 0);

        PlanarJoint() {
            type = Type::Planar;
        }

        void solve(double dt) override;
    };
}
