#pragma once

#include "../phyc.hpp"
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

    struct Surfaces {
        static std::vector<SurfaceTriangleElement> rotate(
            const std::vector<SurfaceTriangleElement>& surfaces,
            const mat3& rotationMatrix
        );
        static std::vector<SurfaceTriangleElement> translate(
            const std::vector<SurfaceTriangleElement>& surfaces,
            const vec3& translation
        );
        static std::vector<SurfaceTriangleElement> rectPrism(
            const vec3& dimensions,
            glm::vec3 color = {0.2f, 0.6f, 0.1f},
            double thickness = 0.007,
            double density = 27.5
        );
        static std::vector<SurfaceTriangleElement> cube(
            float side,
            glm::vec3 color = {0.2f, 0.2f, 0.6f},
            double thickness = 0.007,
            double density = 27.5
        );
        static std::vector<SurfaceTriangleElement> circle(
            float radius,
            int segments = 32,
            glm::vec3 color = {0.6f, 0.2f, 0.2f},
            double thickness = 0.005,
            double density = 27.5
        );
        static std::vector<SurfaceTriangleElement> aroundCylinder(
            float radius,
            float height,
            int segments = 32,
            glm::vec3 color = {0.6f, 0.6f, 0.2f},
            double thickness = 0.005,
            double density = 27.5
        );
        static std::vector<SurfaceTriangleElement> cylinder(
            float radius,
            float height,
            int segments = 32,
            glm::vec3 color = {0.2f, 0.6f, 0.6f},
            double thickness = 0.005,
            double density = 27.5
        );
        static std::vector<SurfaceTriangleElement> aroundCone(
            float baseRadius,
            float height,
            int segments = 32,
            glm::vec3 color = {0.8f, 0.5f, 0.2f},
            double thickness = 0.005,
            double density = 27.5
        );
        static std::vector<SurfaceTriangleElement> arrow(
            const vec3& vec,
            glm::vec3 color = {1.0f, 0.0f, 0.0f},
            double shaftRadius = 0.1,
            double headRadius = 0.2,
            double headLength = 0.5,
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
