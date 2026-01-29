#pragma once
#include "../phyc.hpp"

namespace PhyC {
    struct AirProperties {
        vec3 vel;
        double density;
        double pressure;
        double viscosity;
    };
}
