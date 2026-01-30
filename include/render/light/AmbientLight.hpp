#pragma once
#include "Light.hpp"

namespace PhyCRenderer {
    struct AmbientLight : Light {
        AmbientLight() : Light(LightType::Ambient) {
        }

        void update(Shader& shader) override {
            ambientColor += color * intensity;
        }
    };
}
