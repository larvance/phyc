#pragma once
#include "Controller.hpp"
#include "render/Scene.hpp"

namespace PhyCRenderer {
    struct OrbitController : Controller {
        float orbitSensitivity = 0.005f;
        float panSensitivity = 0.005f;
        float zoomSpeed = 1.5f;
        float dragZoomSpeed = 0.05f;
        float defaultDistance = 10.0f;

        bool isPanning = false;
        bool isOrbiting = false;
        bool isDragZooming = false;
        glm::vec3 pivotPoint{0.0f};
        World& world;
        RigidBody* body = nullptr;

        explicit OrbitController(World& world) : world(world) {
        }

        void updateBody() {
            if (isOrbiting || isDragZooming) {
                if (!body || !world.hasRigidBody(*body)) {
                    body = &world.createRigidBody("OrbitPivot");
                    body->isStatic = true;
                    body->surface = Surfaces::sphere(0.1, 32, 32, {1.0f, 0.0f, 0.0f});
                } else if (body->curState.pos != toEigen(pivotPoint)) {
                    body->curState.pos = toEigen(pivotPoint);
                }
            } else if (body) {
                world.removeRigidBody(*body);
                body = nullptr;
            }
        }

        glm::vec3 getTargetPoint(Scene& scene, double mouseX, double mouseY) const {
            RayHit hit;
            bool hitSuccess = world.raycast(
                hit,
                mouseX, mouseY, scene.viewport[2], scene.viewport[3],
                toEigen(scene.camera.view()),
                toEigen(scene.camera.projection(scene.window.aspectRatio()))
            );

            if (hitSuccess) return toGlm(hit.point);
            return scene.camera.position + (scene.camera.front * defaultDistance);
        }

        void onMouseDown(Scene& scene, int button, int mods) override {
            if (button == GLFW_MOUSE_BUTTON_RIGHT) {
                pivotPoint = getTargetPoint(scene, scene.mouse.x, scene.mouse.y);

                if (mods & GLFW_MOD_CONTROL) {
                    isDragZooming = true;
                } else {
                    isOrbiting = true;
                }

                scene.window.disableCursor();
                scene.mouse.moved = false;
            }

            if (button == GLFW_MOUSE_BUTTON_LEFT) {
                isPanning = true;
                scene.window.disableCursor();
                scene.mouse.moved = false;
            }
        }

        void onMouseUp(Scene& scene, int button, int mods) override {
            if (button == GLFW_MOUSE_BUTTON_RIGHT) {
                isOrbiting = false;
                isDragZooming = false;

                if (!isPanning) scene.window.showCursor();
            }

            if (button == GLFW_MOUSE_BUTTON_LEFT) {
                isPanning = false;

                if (!isOrbiting && !isDragZooming) scene.window.showCursor();
            }
        }

        void onMouseMove(Scene& scene) override {
            if (!scene.mouse.moved) return;

            if (isPanning) {
                glm::vec3 camRight = glm::normalize(glm::cross(scene.camera.front, scene.camera.up));
                glm::vec3 camUp = glm::normalize(glm::cross(camRight, scene.camera.front));

                glm::vec3 offset = (camRight * (float)(-scene.mouse.dx) * panSensitivity) +
                    (camUp * (float)(scene.mouse.dy) * panSensitivity);

                scene.camera.position += offset;
            }

            if (isOrbiting) {
                glm::vec3 arm = scene.camera.position - pivotPoint;

                float yawAngle = -scene.mouse.dx * orbitSensitivity;
                glm::mat4 rot = glm::rotate(glm::mat4(1.0f), yawAngle, glm::vec3(0.0f, 0.0f, 1.0f));
                arm = glm::vec3(rot * glm::vec4(arm, 1.0f));

                glm::vec3 camRight = glm::normalize(glm::cross(scene.camera.front, scene.camera.up));
                float pitchAngle = -scene.mouse.dy * orbitSensitivity;

                glm::mat4 pitchRot = glm::rotate(glm::mat4(1.0f), pitchAngle, camRight);
                glm::vec3 nextArm(pitchRot * glm::vec4(arm, 1.0f));

                float dot = glm::dot(glm::normalize(nextArm), glm::vec3(0.0f, 0.0f, 1.0f));
                if (std::abs(dot) < 0.99f) {
                    arm = nextArm;
                }

                scene.camera.position = pivotPoint + arm;
                scene.camera.lookAt(pivotPoint);
            }

            if (isDragZooming) {
                float zoomAmount = scene.mouse.dy * dragZoomSpeed;
                glm::vec3 dir = pivotPoint - scene.camera.position;
                float dist = glm::length(dir);

                if (!(zoomAmount > 0 && dist < 0.2f)) {
                    scene.camera.position += glm::normalize(dir) * zoomAmount;
                }
            }
        }

        void onScroll(Scene& scene, double dx, double dy) override {
            glm::vec3 target = getTargetPoint(scene, scene.mouse.x, scene.mouse.y);
            glm::vec3 dir = target - scene.camera.position;
            float dist = glm::length(dir);
            float moveAmount = (float)dy * zoomSpeed;

            if (moveAmount > 0) {
                if (dist > 0.5f) {
                    scene.camera.position += glm::normalize(dir) * std::min(moveAmount, dist - 0.5f);
                }
            } else {
                scene.camera.position += glm::normalize(dir) * moveAmount;
            }
        }

        void onFocusOut(Scene& scene) override {
            isPanning = false;
            isOrbiting = false;
            isDragZooming = false;
            scene.window.showCursor();
        }

        void onUpdate(Scene& scene) override {
            updateBody();
        }
    };
}
