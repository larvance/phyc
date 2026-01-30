#pragma once
#include "RigidBody.hpp"

namespace PhyC {
    struct Spring {
        RigidBody* bodyA = nullptr;
        vec3 anchorA{0, 0, 0};

        RigidBody* bodyB = nullptr;
        vec3 anchorB{0, 0, 0};
        vec3 worldTargetPos{0, 0, 0};

        double stiffness = 50.0;
        double damping = 2.0;
        double restLength = 2.0;

        void applyForce(double dt) const;
    };
}
