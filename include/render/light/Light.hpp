#pragma once
#include <imgui.h>
#include <string>
#include <glm/vec3.hpp>
#include "physics/RigidBody.hpp"
#include "render/Shader.hpp"

namespace PhyCRenderer {
    enum class LightType { Directional, Spot, Hemisphere, Point, Ambient };

    static constexpr int LightTypeCount = static_cast<int>(LightType::Ambient) + 1;
    static constexpr std::string LightTypeNames[LightTypeCount] = {
        "Directional", "Spot", "Hemisphere", "Point", "Ambient"
    };
    static constexpr int MaxLightCount = 8;

    struct Light {
        static glm::vec3 ambientColor;
        static int lightIndex;

        unsigned long long id = PhyC::getNextId();
        LightType type;
        glm::vec3 color{1.0f, 1.0f, 1.0f};
        float intensity{1.0f};

        explicit Light(LightType t) : type(t) {
        }

        Light(const Light&) = delete;
        Light& operator=(const Light&) = delete;

        virtual void update(Shader& shader) = 0;

        virtual void gui() {
            ImGui::ColorEdit3("Color", &color[0]);
            ImGui::DragFloat("Intensity", &intensity, 0.1f);
        }

        virtual ~Light() = default;

        static auto base(const std::string& suffix) {
            return "lights[" + std::to_string(lightIndex) + "]." + suffix;
        }
    };
}
