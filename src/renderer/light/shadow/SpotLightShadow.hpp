#pragma once
#include "LightShadow.hpp"

namespace PhyCRenderer {
    struct SpotLightShadow : LightShadow {
        float aspect{1.0f};
        float focus{1.0f};
    };
}
