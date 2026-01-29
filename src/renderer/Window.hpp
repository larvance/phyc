#pragma once
#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

namespace PhyCRenderer {
    struct Window {
        GLFWwindow* window;
        int width, height;
        void* userPointer = nullptr;

        Window(int width, int height) : width(width), height(height) {
            if (!glfwInit()) {
                std::cerr << "Failed to initialize GLFW" << std::endl;
                throw std::runtime_error("Failed to initialize GLFW");
            }
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

            window = glfwCreateWindow(width, height, "PhyC Physics Renderer", nullptr, nullptr);
            if (!window) {
                glfwTerminate();
                std::cerr << "Failed to create window" << std::endl;
                throw std::runtime_error("Failed to create window");
            }
            glfwMakeContextCurrent(window);
            glfwSwapInterval(1);

            if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
                std::cerr << "Failed to initialize GLAD" << std::endl;
                throw std::runtime_error("Failed to initialize GLAD");
            }

            glEnable(GL_DEPTH_TEST);

            glfwSetWindowUserPointer(window, this);
        }

        bool isCursorNormal() const {
            return glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_NORMAL;
        }

        bool isCursorDisabled() const {
            return glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED;
        }

        bool isCursorHidden() const {
            return glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_HIDDEN;
        }

        void showCursor() const {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        }

        void hideCursor() const {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
        }

        void disableCursor() const {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        }

        bool shouldClose() const {
            return glfwWindowShouldClose(window);
        }

        void poll() {
            glfwPollEvents();
            glfwGetFramebufferSize(window, &width, &height);
        }

        void swapBuffers() const {
            glfwSwapBuffers(window);
        }

        ~Window() {
            glfwDestroyWindow(window);
            glfwTerminate();
        }

        static Window& from(GLFWwindow* glfwWindow) {
            return *static_cast<Window*>(glfwGetWindowUserPointer(glfwWindow));
        }
    };
}
