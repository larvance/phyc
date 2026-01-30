#pragma once

#include "phyc.hpp"
#include "glm/glm.hpp"

namespace PhyC {
    struct SurfaceTriangleElement {
        vec3 offset;
        vec3 normal;
        double area;
        double thickness;
        double density;

        mat3 corners;
        glm::vec3 color;

        double mass() const {
            return density * area * thickness;
        }
    };

    struct Surface {
        std::vector<SurfaceTriangleElement> elements;

        void operator +=(const Surface& other) {
            elements.insert(elements.end(), other.elements.begin(), other.elements.end());
        }
    };

    struct Surfaces {
        static Surface rotate(
            const Surface& surface,
            const mat3& rotationMatrix
        );
        static Surface translate(
            const Surface& surfaces,
            const vec3& translation
        );
        static Surface rectPrism(
            const vec3& dimensions,
            glm::vec3 color = {0.2f, 0.6f, 0.1f},
            double thickness = 0.007,
            double density = 27.5
        );
        static Surface cube(
            float side,
            glm::vec3 color = {0.2f, 0.2f, 0.6f},
            double thickness = 0.007,
            double density = 27.5
        );
        static Surface circle(
            float radius,
            int segments = 32,
            glm::vec3 color = {0.6f, 0.2f, 0.2f},
            double thickness = 0.005,
            double density = 27.5
        );
        static Surface aroundCylinder(
            float radius,
            float height,
            int segments = 32,
            glm::vec3 color = {0.6f, 0.6f, 0.2f},
            double thickness = 0.005,
            double density = 27.5
        );
        static Surface cylinder(
            float radius,
            float height,
            int segments = 32,
            glm::vec3 color = {0.2f, 0.6f, 0.6f},
            double thickness = 0.005,
            double density = 27.5
        );
        static Surface aroundCone(
            float baseRadius,
            float height,
            int segments = 32,
            glm::vec3 color = {0.8f, 0.5f, 0.2f},
            double thickness = 0.005,
            double density = 27.5
        );
        static Surface arrow(
            const vec3& vec,
            glm::vec3 color = {1.0f, 0.0f, 0.0f},
            double shaftRadius = 0.1,
            double headRadius = 0.2,
            double headLength = 0.5,
            double thickness = 0.005,
            double density = 27.5
        );
        static Surface sphere(
            float radius,
            int segments = 16,
            int rings = 16,
            glm::vec3 color = {0.8f, 0.2f, 0.8f},
            double thickness = 0.005,
            double density = 27.5
        );
    };

    static double getTriangleArea(const vec3& v0, const vec3& v1, const vec3& v2) {
        return 0.5 * ((v1 - v0).cross(v2 - v0)).norm();
    }

    static double getTriangleArea(const mat3& v) {
        return getTriangleArea(v.col(0), v.col(1), v.col(2));
    }
}
