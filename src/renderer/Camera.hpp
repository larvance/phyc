#pragma once

namespace PhyCRenderer {
    struct Camera {
        glm::vec3 position{0.0f, 0.0f, 5.0f};
        glm::vec3 front{0.0f, 0.0f, -1.0f};
        glm::vec3 up{0.0f, 0.0f, 1.0f};
        float fov = 120.0f;
        float near = 0.1f;
        float far = 100.0f;

        void lookAt(const glm::vec3& target) {
            front = glm::normalize(target - position);
        }

        glm::mat4 view() const {
            return glm::lookAt(position, position + front, up);
        }

        glm::mat4 projection(float aspectRatio) const {
            return glm::perspective(glm::radians(fov), aspectRatio, near, far);
        }
    };
}
