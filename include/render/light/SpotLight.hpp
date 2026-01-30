#pragma once
#include "Light.hpp"
#include "shadow/SpotLightShadow.hpp"

namespace PhyCRenderer {
    struct SpotLight : Light {
        float angle{glm::radians(30.0f)};
        float decay{2.0f};
        float distance{0.0f}; // 0 = infinite
        float penumbra{0.0f}; // 0 to 1
        float power{1.0f};    // lumens

        SpotLightShadow shadow;
        glm::vec3 position{0.0f, 10.0f, 0.0f};
        glm::vec3 target{0.0f, 0.0f, 0.0f};

        SpotLight() : Light(LightType::Spot) {
        }

        void update(Shader& shader) override {
            if (lightIndex >= MaxLightCount) return;
            shader.set(base("type"), 1);
            shader.set(base("position"), position);
            shader.set(base("direction"), getDirection());
            shader.set(base("color"), color);
            shader.set(base("intensity"), intensity);
            shader.set(base("distance"), distance);
            shader.set(base("decay"), decay);

            float outerCos = cos(angle);
            float innerCos = cos(angle * (1.0f - penumbra));
            shader.set(base("coneCos"), outerCos);
            shader.set(base("penumbraCos"), innerCos);
            lightIndex++;
        }

        void gui() override {
            Light::gui();
            ImGui::DragFloat3("Position", &position[0], 0.1f);
            ImGui::DragFloat3("Target", &target[0], 0.1f);
            ImGui::DragFloat("Angle", &angle, 0.01f, 0.0f, glm::pi<float>() / 2.0f);
            ImGui::DragFloat("Penumbra", &penumbra, 0.01f, 0.0f, 1.0f);
            ImGui::DragFloat("Distance", &distance, 0.1f, 0.0f, 1000.0f);
            ImGui::DragFloat("Decay", &decay, 0.1f, 0.0f, 5.0f);
        }

        glm::vec3 getDirection() const {
            return glm::normalize(target - position);
        }
    };
}
