#pragma once
#include "../phyc.hpp"

namespace PhyC {
    struct CollidingMaterial {
        double restitution = 0.5;
        double friction = 0.6;
        double stiffness = 10000.0;
        double damping = 100.0;
    };

    struct OBB {
        // Standard generic plastic (ABS/PVC). Fairly rigid, average friction,
        // minimal bounce. Critically damped to stop vibration quickly.
        static constexpr CollidingMaterial HardPlastic{
            .restitution = 0.15,
            .friction = 0.4,
            .stiffness = 30000.0,
            .damping = 350.0
        };

        // Extremely stiff and lightweight. Very low restitution (brittle/rigid),
        // smooth finish (lower friction). Low damping allows for high-frequency
        // vibrations (the "clack" sound on impact).
        static constexpr CollidingMaterial CarbonFiber{
            .restitution = 0.1,
            .friction = 0.3,
            .stiffness = 85000.0,
            .damping = 150.0
        };

        // High friction, soft (low stiffness), and retains energy (bouncy).
        // Damping is kept low to allow the "jiggle" or bounce of rubber.
        static constexpr CollidingMaterial Rubber{
            .restitution = 0.75,
            .friction = 0.9,
            .stiffness = 4000.0,
            .damping = 40.0
        };

        // Designed specifically to Eat Energy. Low restitution (we don't want to bounce
        // back into the air), high friction (tires), medium stiffness, but
        // EXTREMELY high damping to act as a shock absorber.
        static constexpr CollidingMaterial LandingGear{
            .restitution = 0.05,
            .friction = 0.8,
            .stiffness = 18000.0,
            .damping = 2500.0
        };

        // Very lightweight and porous. High friction (texture), very soft (low stiffness).
        // Good damping relative to its stiffness to absorb crash energy without shattering.
        static constexpr CollidingMaterial EPOFoam{
            .restitution = 0.2,
            .friction = 0.6,
            .stiffness = 2500.0,
            .damping = 110.0
        };

        vec3 centerOffset = vec3::Zero();
        vec3 halfExtents = vec3::Zero();
        mat3 localBasis = mat3::Identity();
        CollidingMaterial material = HardPlastic;

        static OBB simple(const vec3& size, const vec3& offset = vec3::Zero(),
                          const quaternion& localRot = quaternion::Identity()) {
            return {offset, size * 0.5, localRot.toRotationMatrix()};
        }

        static OBB cube(double sideLength, const vec3& offset = vec3::Zero(),
                        const quaternion& localRot = quaternion::Identity()) {
            return {offset, vec3{sideLength * 0.5, sideLength * 0.5, sideLength * 0.5}, localRot.toRotationMatrix()};
        }

        constexpr OBB withMaterial(const CollidingMaterial& mat) const {
            OBB box = *this;
            box.material = mat;
            return box;
        }
    };
}
