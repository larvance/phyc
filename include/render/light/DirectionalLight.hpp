#pragma once
#include "Light.hpp"
#include "shadow/LightShadow.hpp"

namespace PhyCRenderer {
    struct DirectionalLight : Light {
        LightShadow shadow;
        glm::vec3 position{0.0f, 10.0f, 0.0f};
        glm::vec3 target{0.0f, 0.0f, 0.0f};

        DirectionalLight() : Light(LightType::Directional) {
        }

        void update(Shader& shader) override {
            if (lightIndex >= MaxLightCount) return;
            shader.set(base("type"), 0);
            shader.set(base("direction"), getDirection());
            shader.set(base("color"), color);
            shader.set(base("intensity"), intensity);
            lightIndex++;
        }

        void gui() override {
            Light::gui();
            ImGui::DragFloat3("Position", &position[0], 0.1f);
            ImGui::DragFloat3("Target", &target[0], 0.1f);
        }

        glm::vec3 getDirection() const {
            return glm::normalize(target - position);
        }
    };
}
