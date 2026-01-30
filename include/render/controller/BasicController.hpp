#pragma once
#include "Controller.hpp"

namespace PhyCRenderer {
    struct BasicController : Controller {
        float speed = 10.0f;
        float sensitivity = 0.1f;

        void onMouseMove(Scene& scene) override {
            if (!scene.window.isCursorDisabled()) return;

            glm::vec3 front = glm::normalize(scene.camera.front);

            float pitch = glm::degrees(asin(glm::clamp(front.z, -1.0f, 1.0f)));

            float yaw = glm::degrees(atan2(front.y, front.x));

            yaw -= scene.mouse.dx * sensitivity;
            pitch -= scene.mouse.dy * sensitivity;

            if (pitch > 89.0f) pitch = 89.0f;
            if (pitch < -89.0f) pitch = -89.0f;

            glm::vec3 newFront;
            newFront.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
            newFront.y = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
            newFront.z = sin(glm::radians(pitch));

            scene.camera.front = glm::normalize(newFront);
        }

        void onMouseDown(Scene& scene, int button, int mods) override {
            if (button == GLFW_MOUSE_BUTTON_LEFT && scene.window.isCursorNormal()) {
                scene.window.disableCursor();
                scene.mouse.moved = false;
            }
        }

        void onKeyDown(Scene& scene, int key, int scancode, int mods) override {
            if (key == GLFW_KEY_ESCAPE && scene.window.isCursorDisabled()) {
                scene.window.showCursor();
            }
        }

        void onFocusOut(Scene& scene) override {
            scene.window.showCursor();
        }

        void onUpdate(Scene& scene) override {
            glm::vec3 flatFront = glm::normalize(glm::vec3(scene.camera.front.x, scene.camera.front.y, 0.0f));
            glm::vec3 cameraRight = glm::normalize(glm::cross(scene.camera.front, scene.camera.up));

            float velocity = speed * scene.deltaTime;

            if (scene.keyboard[GLFW_KEY_W]) scene.camera.position += flatFront * velocity;
            if (scene.keyboard[GLFW_KEY_S]) scene.camera.position -= flatFront * velocity;
            if (scene.keyboard[GLFW_KEY_A]) scene.camera.position -= cameraRight * velocity;
            if (scene.keyboard[GLFW_KEY_D]) scene.camera.position += cameraRight * velocity;

            if (scene.keyboard[GLFW_KEY_SPACE]) scene.camera.position += scene.camera.up * velocity;
            if (scene.keyboard[GLFW_KEY_LEFT_SHIFT]) scene.camera.position -= scene.camera.up * velocity;
        }
    };
}
