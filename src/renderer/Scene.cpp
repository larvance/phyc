#include "Scene.hpp"

#include "ShaderStrings.hpp"

using namespace PhyCRenderer;

Scene::Scene(Window& window) : window(window) {
    fragShader = fragmentShaderSource;
    updateFragLightCount();

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);

    window.userPointer = this;

    glfwSetKeyCallback(window.window, [](GLFWwindow* _, int key, int scancode, int action, int mods) {
        auto& scene = from(_);

        if (key >= 0 && key < GLFW_KEY_LAST) {
            if (action == GLFW_PRESS) {
                scene.keyboard[key] = true;
            } else if (action == GLFW_RELEASE) {
                scene.keyboard[key] = false;
            }
        }

        for (auto& controller : scene.controllers) {
            if (action == GLFW_PRESS) {
                controller->onKeyDown(scene, key, scancode, mods);
            } else if (action == GLFW_RELEASE) {
                controller->onKeyUp(scene, key, scancode, mods);
            } else if (action == GLFW_REPEAT) {
                controller->onKeyPress(scene, key, scancode, mods);
            }
        }
    });
    glfwSetFramebufferSizeCallback(window.window, [](GLFWwindow* _, int width, int height) {
        auto& scene = from(_);

        for (auto& controller : scene.controllers) {
            controller->onResize(scene);
        }
    });
    glfwSetCursorPosCallback(window.window, [](GLFWwindow* _, double x, double y) {
        auto& scene = from(_);
        x -= scene.viewport[0];
        y -= scene.viewport[1];
        if (!scene.mouse.moved) {
            scene.mouse.moved = true;
            scene.mouse.lastX = x;
            scene.mouse.lastY = y;
            scene.mouse.dx = 0;
            scene.mouse.dy = 0;
        } else {
            scene.mouse.lastX = scene.mouse.x;
            scene.mouse.lastY = scene.mouse.y;
            scene.mouse.dx = x - scene.mouse.x;
            scene.mouse.dy = y - scene.mouse.y;
        }
        scene.mouse.x = x;
        scene.mouse.y = y;

        for (auto& controller : scene.controllers) {
            controller->onMouseMove(scene);
        }
    });
    glfwSetMouseButtonCallback(window.window, [](GLFWwindow* _, int button, int action, int mods) {
        auto& scene = from(_);

        bool new_val = ((action != GLFW_RELEASE));
        if (new_val == scene.mouse.buttons[button]) return;
        if (scene.mouse.x < 0
            || scene.mouse.y < 0
            || scene.mouse.x > scene.viewport[2]
            || scene.mouse.y > scene.viewport[3]) {
            return;
        }
        scene.mouse.buttons[button] = new_val;

        for (auto& controller : scene.controllers) {
            if (action == GLFW_PRESS) {
                scene.mouse.downX = scene.mouse.x;
                scene.mouse.downY = scene.mouse.y;
                controller->onMouseDown(scene, button, mods);
            } else if (action == GLFW_RELEASE) {
                controller->onMouseUp(scene, button, mods);
                if (
                    std::abs(scene.mouse.x - scene.mouse.downX) < 5 &&
                    std::abs(scene.mouse.y - scene.mouse.downY) < 5
                ) {
                    controller->onMouseClick(scene, button);
                }
            }
        }
    });
    glfwSetScrollCallback(window.window, [](GLFWwindow* _, double dx, double dy) {
        auto& scene = from(_);

        for (auto& controller : scene.controllers) {
            controller->onScroll(scene, dx, dy);
        }
    });
    glfwSetWindowFocusCallback(window.window, [](GLFWwindow* _, int focused) {
        auto& scene = from(_);

        scene.focused = (focused == GLFW_TRUE);

        for (auto& controller : scene.controllers) {
            if (scene.focused) controller->onFocusIn(scene);
            else controller->onFocusOut(scene);
        }
    });
}

void Scene::updateFragLightCount() {
    if (lastNumLights == lights.size()) return;
    lastNumLights = lights.size();

    const std::string headerText = "#define MAX_LIGHTS ";

    fragShader = fragmentShaderSource;

    size_t startPos = fragShader.find(headerText);
    if (startPos != std::string::npos) {
        size_t numberPos = startPos + headerText.length();

        size_t endPos = fragShader.find_first_not_of("0123456789", numberPos);
        size_t lengthToRemove = (endPos == std::string::npos) ? std::string::npos : (endPos - numberPos);

        std::string newCount = std::to_string(std::max(lights.size(), 2UL));

        fragShader.replace(numberPos, lengthToRemove, newCount);
    }

    shader.load(vertexShaderSource, fragShader.c_str());
}

Scene::~Scene() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void Scene::poll() {
    float currentFrame = static_cast<float>(glfwGetTime());
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    window.poll();

    for (auto& controller : controllers) {
        controller->onUpdate(*this);
    }

    updateFragLightCount();
}

void Scene::render(const World& world) {
    if (viewport[2] <= 0 || viewport[3] <= 0) return;
    if (viewport[2] == -1) viewport[2] = window.width;
    if (viewport[3] == -1) viewport[3] = window.height;
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glClearColor(backgroundColor.r, backgroundColor.g, backgroundColor.b, backgroundColor.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);

    if (world.bodies.empty()) return;

    shader.use();

    float aspect = viewport[2] / viewport[3];
    glm::mat4 view = camera.view();
    glm::mat4 proj = camera.projection(aspect);

    shader.set("view", view);
    shader.set("projection", proj);
    shader.set("viewPos", camera.position);

    shader.use();

    Light::ambientColor = glm::vec3(0.0f);
    Light::lightIndex = 0;

    for (const auto& light : lights) {
        light->update(shader);
    }

    shader.set("numLights", Light::lightIndex);
    shader.set("ambientLightColor", Light::ambientColor);

    glBindVertexArray(VAO);

    auto model = shader.get("model");
    auto normalMatrix = shader.get("normalMatrix");

    for (size_t i = 0; i < world.bodies.size(); ++i) {
        const auto& body = *world.bodies[i];

        glm::vec3 bodyPos = toGlm(body.curState.pos);
        mat3 rot = body.curState.rot.toRotationMatrix();

        glm::mat4 rotationMatrix(1.0f);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                rotationMatrix[c][r] = rot(r, c);

        auto modelVal = glm::translate(glm::mat4(1.0f), bodyPos) * rotationMatrix;
        model.set(modelVal);
        normalMatrix.set(glm::transpose(glm::inverse(glm::mat3(modelVal))));

        vertexBuffer.clear();
        for (const auto& surface : body.surface) {
            glm::vec3 normal = toGlm(surface.normal);
            glm::mat3 corners = toGlm(surface.corners);

            vertexBuffer.push_back({corners[0], normal, surface.color});
            vertexBuffer.push_back({corners[1], normal, surface.color});
            vertexBuffer.push_back({corners[2], normal, surface.color});
        }

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertexBuffer.size() * sizeof(Vertex), vertexBuffer.data(),
                     GL_DYNAMIC_DRAW);

        glDrawArrays(GL_TRIANGLES, 0, vertexBuffer.size());
    }

    glBindVertexArray(0);
}

void Scene::addLight(const std::shared_ptr<Light>& light) {
    lights.push_back(light);
}

void Scene::addController(const std::shared_ptr<Controller>& controller) {
    controllers.push_back(controller);
}

Scene& Scene::from(GLFWwindow* glfwWindow) {
    auto window = static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
    return *static_cast<Scene*>(window->userPointer);
}
