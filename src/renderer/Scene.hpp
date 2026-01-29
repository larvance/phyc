#pragma once
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>

#include "../physics/World.hpp"
#include "Camera.hpp"
#include "controller/Controller.hpp"
#include "Shader.hpp"
#include "Window.hpp"
#include "light/Light.hpp"

namespace PhyCRenderer {
    using namespace PhyC;

    struct RenderConfig {
        glm::vec3 color{0.7f, 0.7f, 0.9f};
    };

    struct Mouse {
        double lastX = 0.0, lastY = 0.0;
        double x = 0.0, y = 0.0;
        double dx = 0.0, dy = 0.0;
        bool buttons[8] = {false};
        double downX = 0.0, downY = 0.0;
        bool moved = false;
    };

    struct Scene {
        std::string fragShader;
        std::vector<std::shared_ptr<Light>> lights;
        Window& window;
        std::vector<std::shared_ptr<Controller>> controllers;
        Mouse mouse;
        Camera camera;
        bool keyboard[GLFW_KEY_LAST] = {false};
        bool focused = true;
        glm::vec4 backgroundColor{0.1f, 0.1f, 0.15f, 1.0f};
        float deltaTime = 0.0f;
        float lastFrame = 0.0f;
        glm::vec4 viewport = {0, 0, -1, -1};
        int lastNumLights = -1;

        Shader shader{};
        GLuint VAO, VBO;

        struct Vertex {
            glm::vec3 position;
            glm::vec3 normal;
            glm::vec3 color;
        };

        std::vector<Vertex> vertexBuffer;

        explicit Scene(Window& window);
        void updateFragLightCount();
        ~Scene();
        void poll();
        void render(const World& world);
        void addLight(const std::shared_ptr<Light>& light);
        void addController(const std::shared_ptr<Controller>& controller);
        static Scene& from(GLFWwindow* glfwWindow);
    };
}
