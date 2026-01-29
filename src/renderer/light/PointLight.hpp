#pragma once
#include "Light.hpp"
#include "shadow/LightShadow.hpp"

namespace PhyCRenderer {
    struct PointLight : Light {
        float distance{0.0f}; // 0 = infinite
        float decay{1.0f};

        LightShadow shadow;
        glm::vec3 position{0.0f, 0.0f, 0.0f};

        PointLight() : Light(LightType::Point) {
        }

        void update(Shader& shader) override {
            if (lightIndex >= MaxLightCount) return;
            shader.set(base("type"), 3);
            shader.set(base("position"), position);
            shader.set(base("color"), color);
            shader.set(base("intensity"), intensity);
            shader.set(base("distance"), distance);
            shader.set(base("decay"), decay);
            lightIndex++;
        }

        void gui() override {
            Light::gui();
            ImGui::DragFloat3("Position", &position[0], 0.1f);
            ImGui::DragFloat("Distance", &distance, 0.1f, 0.0f, 1000.0f);
            ImGui::DragFloat("Decay", &decay, 0.1f, 0.0f, 5.0f);
        }
    };
}
