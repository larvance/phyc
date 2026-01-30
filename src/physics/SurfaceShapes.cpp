#include "physics/SurfaceTriangleElement.hpp"

using namespace PhyC;

Surface Surfaces::rotate(
    const Surface& surface,
    const mat3& rotationMatrix
) {
    Surface rotatedSurface;
    rotatedSurface.elements.reserve(surface.elements.size());

    for (const auto& s : surface.elements) {
        mat3 rotatedCorners;
        for (int i = 0; i < 3; i++) rotatedCorners.col(i) = rotationMatrix * s.corners.col(i);

        vec3 rotatedNormal = rotationMatrix * s.normal;

        rotatedSurface.elements.emplace_back(
            s.offset,
            rotatedNormal,
            s.area,
            s.thickness,
            s.density,
            rotatedCorners,
            s.color
        );
    }

    return rotatedSurface;
}

Surface Surfaces::translate(
    const Surface& surfaces,
    const vec3& translation
) {
    Surface translatedSurfaces;
    translatedSurfaces.elements.reserve(surfaces.elements.size());

    for (const auto& s : surfaces.elements) {
        mat3 translatedCorners;
        for (int i = 0; i < 3; i++) {
            translatedCorners.col(i) = s.corners.col(i) + translation;
        }

        translatedSurfaces.elements.emplace_back(
            s.offset + translation,
            s.normal,
            s.area,
            s.thickness,
            s.density,
            translatedCorners,
            s.color
        );
    }

    return translatedSurfaces;
}

Surface Surfaces::rectPrism(
    const vec3& dimensions,
    glm::vec3 color,
    double thickness,
    double density
) {
    Surface surfaces;
    vec3 h = dimensions / 2.0;

    vec3 p[8] = {
        {-h.x(), -h.y(), -h.z()}, {h.x(), -h.y(), -h.z()}, {h.x(), h.y(), -h.z()}, {-h.x(), h.y(), -h.z()},
        {-h.x(), -h.y(), h.z()}, {h.x(), -h.y(), h.z()}, {h.x(), h.y(), h.z()}, {-h.x(), h.y(), h.z()}
    };

    auto addFace = [&](int i0, int i1, int i2, vec3 normal) {
        mat3 corners;
        corners.col(0) = p[i0];
        corners.col(1) = p[i1];
        corners.col(2) = p[i2];

        double area = getTriangleArea(corners);
        surfaces.elements.emplace_back(vec3::Zero(), normal, area, thickness, density, corners, color);
    };

    addFace(0, 3, 2, {0, 0, -1});
    addFace(0, 2, 1, {0, 0, -1});
    addFace(4, 5, 6, {0, 0, 1});
    addFace(4, 6, 7, {0, 0, 1});
    addFace(0, 1, 5, {0, -1, 0});
    addFace(0, 5, 4, {0, -1, 0});
    addFace(2, 3, 7, {0, 1, 0});
    addFace(2, 7, 6, {0, 1, 0});
    addFace(0, 4, 7, {-1, 0, 0});
    addFace(0, 7, 3, {-1, 0, 0});
    addFace(1, 2, 6, {1, 0, 0});
    addFace(1, 6, 5, {1, 0, 0});
    return surfaces;
}

Surface Surfaces::cube(
    float side,
    glm::vec3 color,
    double thickness,
    double density
) {
    return rectPrism({side, side, side}, color, thickness, density);
}

Surface Surfaces::circle(
    float radius,
    int segments,
    glm::vec3 color,
    double thickness,
    double density
) {
    Surface surfaces;
    vec3 center = {0.0, 0.0, 0.0};
    for (int i = 0; i < segments; i++) {
        float theta0 = (2.0f * M_PI * i) / segments;
        float theta1 = (2.0f * M_PI * (i + 1)) / segments;
        vec3 p0 = {radius * cos(theta0), radius * sin(theta0), 0.0};
        vec3 p1 = {radius * cos(theta1), radius * sin(theta1), 0.0};

        mat3 corners;
        corners.col(0) = center;
        corners.col(1) = p0;
        corners.col(2) = p1;

        double area = getTriangleArea(corners);
        surfaces.elements.emplace_back(
            vec3{0.0, 0.0, 0.0},
            vec3{0.0, 0.0, 1.0},
            area,
            thickness,
            density,
            corners,
            color
        );
    }
    return surfaces;
}

Surface Surfaces::aroundCylinder(
    float radius,
    float height,
    int segments,
    glm::vec3 color,
    double thickness,
    double density
) {
    Surface surfaces;
    float h = height / 2.0f;
    for (int i = 0; i < segments; i++) {
        float theta0 = (2.0f * M_PI * i) / segments;
        float theta1 = (2.0f * M_PI * (i + 1)) / segments;
        vec3 p0_top = {radius * cos(theta0), radius * sin(theta0), h};
        vec3 p1_top = {radius * cos(theta1), radius * sin(theta1), h};
        vec3 p0_bottom = {radius * cos(theta0), radius * sin(theta0), -h};
        vec3 p1_bottom = {radius * cos(theta1), radius * sin(theta1), -h};

        mat3 corners1;
        corners1.col(0) = p0_bottom;
        corners1.col(1) = p1_bottom;
        corners1.col(2) = p1_top;

        double area1 = getTriangleArea(corners1);
        surfaces.elements.emplace_back(
            vec3{0.0, 0.0, 0.0},
            (p1_bottom - p0_bottom).cross(p1_top - p0_bottom).normalized(),
            area1,
            thickness,
            density,
            corners1,
            color
        );

        mat3 corners2;
        corners2.col(0) = p0_bottom;
        corners2.col(1) = p1_top;
        corners2.col(2) = p0_top;

        double area2 = getTriangleArea(corners2);
        surfaces.elements.emplace_back(
            vec3{0.0, 0.0, 0.0},
            (p1_top - p0_bottom).cross(p0_top - p0_bottom).normalized(),
            area2,
            thickness,
            density,
            corners2,
            color
        );
    }
    return surfaces;
}

Surface Surfaces::cylinder(
    float radius,
    float height,
    int segments,
    glm::vec3 color,
    double thickness,
    double density
) {
    Surface surfaces = aroundCylinder(
        radius, height, segments, color, thickness, density
    );

    auto topFaces = translate(
        circle(radius, segments, color, thickness, density),
        vec3{0.0, 0.0, height / 2.0}
    );
    auto bottomFaces = translate(
        circle(radius, segments, color, thickness, density),
        vec3{0.0, 0.0, -height / 2.0}
    );
    for (auto& f : bottomFaces.elements) {
        f.normal = vec3{0.0, 0.0, -1.0};
    }

    surfaces += topFaces;
    surfaces += bottomFaces;

    return surfaces;
}

Surface Surfaces::aroundCone(
    float baseRadius,
    float height,
    int segments,
    glm::vec3 color,
    double thickness,
    double density
) {
    Surface surfaces;
    vec3 apex = {0.0, 0.0, height / 2.0};

    for (int i = 0; i < segments; i++) {
        float theta0 = (2.0f * M_PI * i) / segments;
        float theta1 = (2.0f * M_PI * (i + 1)) / segments;
        vec3 p0 = {baseRadius * cos(theta0), baseRadius * sin(theta0), -height / 2.0};
        vec3 p1 = {baseRadius * cos(theta1), baseRadius * sin(theta1), -height / 2.0};

        mat3 corners;
        corners.col(0) = apex;
        corners.col(1) = p0;
        corners.col(2) = p1;

        double area = getTriangleArea(corners);

        vec3 normal = (p0 - apex).cross(p1 - apex).normalized();

        surfaces.elements.emplace_back(
            vec3{0.0, 0.0, 0.0},
            normal,
            area,
            thickness,
            density,
            corners,
            color
        );
    }

    return surfaces;
}

Surface Surfaces::arrow(
    const vec3& vec,
    glm::vec3 color,
    double shaftRadius,
    double headRadius,
    double headLength,
    double thickness,
    double density
) {
    Surface surfaces;
    double vecLength = vec.norm();
    if (vecLength < 1e-6) return surfaces;

    vec3 dir = vec.normalized();
    double shaftLength = vecLength - headLength;
    if (shaftLength < 0.0) {
        shaftLength = 0.0;
        headLength = vecLength;
    }

    surfaces = aroundCylinder(shaftRadius, shaftLength, 12, color, thickness, density);

    auto shaftBottom = circle(shaftRadius, 12, color, thickness, density);
    shaftBottom = translate(shaftBottom, vec3{0.0, 0.0, -shaftLength / 2.0});

    for (auto& f : shaftBottom.elements) {
        f.normal = vec3{0.0, 0.0, -1.0};
        f.corners.col(1).swap(f.corners.col(2));
    }
    surfaces += shaftBottom;

    surfaces = translate(surfaces, vec3{0.0, 0.0, shaftLength / 2.0});

    auto headSurfaces = aroundCone(headRadius, headLength, 12, color, thickness, density);

    auto coneBottom = circle(headRadius, 12, color, thickness, density);
    coneBottom = translate(coneBottom, vec3{0.0, 0.0, -headLength / 2.0});

    for (auto& f : coneBottom.elements) {
        f.normal = vec3{0.0, 0.0, -1.0};
        f.corners.col(1).swap(f.corners.col(2));
    }
    headSurfaces += coneBottom;

    headSurfaces = translate(headSurfaces, vec3{0.0, 0.0, shaftLength + headLength / 2.0});

    surfaces += headSurfaces;

    vec3 zAxis = vec3::UnitZ();
    quaternion q = quaternion::FromTwoVectors(zAxis, dir);
    surfaces = rotate(surfaces, q.toRotationMatrix());

    return surfaces;
}

Surface Surfaces::sphere(float radius, int segments, int rings, glm::vec3 color,
                         double thickness, double density) {
    Surface surfaces;
    for (int r = 0; r < rings; r++) {
        float theta0 = (M_PI * r) / rings;
        float theta1 = (M_PI * (r + 1)) / rings;
        for (int s = 0; s < segments; s++) {
            float phi0 = (2.0f * M_PI * s) / segments;
            float phi1 = (2.0f * M_PI * (s + 1)) / segments;
            vec3 p0 = {
                radius * sin(theta0) * cos(phi0),
                radius * sin(theta0) * sin(phi0),
                radius * cos(theta0)
            };
            vec3 p1 = {
                radius * sin(theta1) * cos(phi0),
                radius * sin(theta1) * sin(phi0),
                radius * cos(theta1)
            };
            vec3 p2 = {
                radius * sin(theta1) * cos(phi1),
                radius * sin(theta1) * sin(phi1),
                radius * cos(theta1)
            };
            vec3 p3 = {
                radius * sin(theta0) * cos(phi1),
                radius * sin(theta0) * sin(phi1),
                radius * cos(theta0)
            };
            mat3 corners1;
            corners1.col(0) = p0;
            corners1.col(1) = p1;
            corners1.col(2) = p2;
            double area1 = getTriangleArea(corners1);
            surfaces.elements.emplace_back(
                vec3{0.0, 0.0, 0.0},
                (p1 - p0).cross(p2 - p0).normalized(),
                area1,
                thickness,
                density,
                corners1,
                color
            );
            mat3 corners2;
            corners2.col(0) = p0;
            corners2.col(1) = p2;
            corners2.col(2) = p3;
            double area2 = getTriangleArea(corners2);
            surfaces.elements.emplace_back(
                vec3{0.0, 0.0, 0.0},
                (p2 - p0).cross(p3 - p0).normalized(),
                area2,
                thickness,
                density,
                corners2,
                color
            );
        }
    }
    return surfaces;
}
