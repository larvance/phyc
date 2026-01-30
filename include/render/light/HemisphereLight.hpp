#pragma once
#include "Light.hpp"

namespace PhyCRenderer {
    struct HemisphereLight : Light {
        glm::vec3& skyColor{color};
        glm::vec3 groundColor{0.2f, 0.2f, 0.2f};

        HemisphereLight() : Light(LightType::Hemisphere) {
        }

        void update(Shader& shader) override {
            if (lightIndex >= MaxLightCount) return;
            shader.set(base("type"), 2);
            shader.set(base("skyColor"), skyColor);
            shader.set(base("groundColor"), groundColor);
            shader.set(base("intensity"), intensity);
            static glm::vec3 up(0.0f, 0.0f, 1.0f);
            shader.set(base("direction"), up);
            lightIndex++;
        }

        void gui() override {
            ImGui::ColorEdit3("Sky Color", &skyColor[0]);
            ImGui::DragFloat("Intensity", &intensity, 0.1f);
            ImGui::ColorEdit3("Ground Color", &groundColor[0]);
        }
    };
}
