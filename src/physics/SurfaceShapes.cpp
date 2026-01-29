#include "SurfaceTriangleElement.hpp"

using namespace PhyC;

std::vector<SurfaceTriangleElement> Surfaces::rotate(
    const std::vector<SurfaceTriangleElement>& surfaces,
    const mat3& rotationMatrix
) {
    std::vector<SurfaceTriangleElement> rotatedSurfaces;
    rotatedSurfaces.reserve(surfaces.size());

    for (const auto& s : surfaces) {
        mat3 rotatedCorners;
        for (int i = 0; i < 3; i++) rotatedCorners.col(i) = rotationMatrix * s.corners.col(i);

        vec3 rotatedNormal = rotationMatrix * s.normal;

        rotatedSurfaces.emplace_back(
            s.offset,
            rotatedNormal,
            s.area,
            s.thickness,
            s.density,
            rotatedCorners,
            s.color
        );
    }

    return rotatedSurfaces;
}

std::vector<SurfaceTriangleElement> Surfaces::translate(
    const std::vector<SurfaceTriangleElement>& surfaces,
    const vec3& translation
) {
    std::vector<SurfaceTriangleElement> translatedSurfaces;
    translatedSurfaces.reserve(surfaces.size());

    for (const auto& s : surfaces) {
        mat3 translatedCorners;
        for (int i = 0; i < 3; i++) {
            translatedCorners.col(i) = s.corners.col(i) + translation;
        }

        translatedSurfaces.emplace_back(
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

std::vector<SurfaceTriangleElement> Surfaces::rectPrism(
    const vec3& dimensions,
    glm::vec3 color,
    double thickness,
    double density
) {
    std::vector<SurfaceTriangleElement> surfaces;
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
        surfaces.emplace_back(vec3::Zero(), normal, area, thickness, density, corners, color);
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

std::vector<SurfaceTriangleElement> Surfaces::cube(
    float side,
    glm::vec3 color,
    double thickness,
    double density
) {
    return rectPrism({side, side, side}, color, thickness, density);
}

std::vector<SurfaceTriangleElement> Surfaces::circle(
    float radius,
    int segments,
    glm::vec3 color,
    double thickness,
    double density
) {
    std::vector<SurfaceTriangleElement> surfaces;
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
        surfaces.emplace_back(
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

std::vector<SurfaceTriangleElement> Surfaces::aroundCylinder(
    float radius,
    float height,
    int segments,
    glm::vec3 color,
    double thickness,
    double density
) {
    std::vector<SurfaceTriangleElement> surfaces;
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
        surfaces.emplace_back(
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
        surfaces.emplace_back(
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

std::vector<SurfaceTriangleElement> Surfaces::cylinder(
    float radius,
    float height,
    int segments,
    glm::vec3 color,
    double thickness,
    double density
) {
    std::vector<SurfaceTriangleElement> surfaces = aroundCylinder(
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
    for (auto& f : bottomFaces) {
        f.normal = vec3{0.0, 0.0, -1.0};
    }

    surfaces.insert(surfaces.end(), topFaces.begin(), topFaces.end());
    surfaces.insert(surfaces.end(), bottomFaces.begin(), bottomFaces.end());

    return surfaces;
}

std::vector<SurfaceTriangleElement> Surfaces::aroundCone(
    float baseRadius,
    float height,
    int segments,
    glm::vec3 color,
    double thickness,
    double density
) {
    std::vector<SurfaceTriangleElement> surfaces;
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

        surfaces.emplace_back(
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

std::vector<SurfaceTriangleElement> Surfaces::arrow(
    const vec3& vec,
    glm::vec3 color,
    double shaftRadius,
    double headRadius,
    double headLength,
    double thickness,
    double density
) {
    std::vector<SurfaceTriangleElement> surfaces;
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

    for (auto& f : shaftBottom) {
        f.normal = vec3{0.0, 0.0, -1.0};
        f.corners.col(1).swap(f.corners.col(2));
    }
    surfaces.insert(surfaces.end(), shaftBottom.begin(), shaftBottom.end());

    surfaces = translate(surfaces, vec3{0.0, 0.0, shaftLength / 2.0});

    auto headSurfaces = aroundCone(headRadius, headLength, 12, color, thickness, density);

    auto coneBottom = circle(headRadius, 12, color, thickness, density);
    coneBottom = translate(coneBottom, vec3{0.0, 0.0, -headLength / 2.0});

    for (auto& f : coneBottom) {
        f.normal = vec3{0.0, 0.0, -1.0};
        f.corners.col(1).swap(f.corners.col(2));
    }
    headSurfaces.insert(headSurfaces.end(), coneBottom.begin(), coneBottom.end());

    headSurfaces = translate(headSurfaces, vec3{0.0, 0.0, shaftLength + headLength / 2.0});

    surfaces.insert(surfaces.end(), headSurfaces.begin(), headSurfaces.end());

    vec3 zAxis = vec3::UnitZ();
    quaternion q = quaternion::FromTwoVectors(zAxis, dir);
    surfaces = rotate(surfaces, q.toRotationMatrix());

    return surfaces;
}
