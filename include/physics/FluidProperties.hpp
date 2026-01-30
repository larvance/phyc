#pragma once
#include "phyc.hpp"

namespace PhyC {
    struct FluidProperties {
        vec3 vel;
        double density;
        double pressure;
        double viscosity;
    };
}
