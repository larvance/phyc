#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>

#include "../utils/ShadowMapResources.hpp"

namespace PhyCRenderer {
    struct LightShadow {
        bool autoUpdate{true};
        float bias{0.005f};
        float blurSamples{8.0f};
        float intensity{1.0f};

        ShadowMapResources mapResources;

        glm::vec2 mapSize{512.0f, 512.0f};
        GLenum mapType{GL_UNSIGNED_BYTE}; // GL_FLOAT for more precision
        glm::mat4 matrix;
        bool needsUpdate{false};
        float normalBias{0.0f};
        float radius{1.0f};
    };
}
